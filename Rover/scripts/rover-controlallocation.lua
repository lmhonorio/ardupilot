--@param control_output integer - CONTROLE MODE 4
--| '1' # Roll
--| '2' # Pitch
--| '3' # Throttle
--| '4' # Yaw
--| '5' # Lateral
--| '6' # MainSail
--| '7' # WingSail
--| '8' # Walking_Height
--@return number|nil
-- External modules
package.path = package.path .. ';./scripts/modules/?.lua'
local PID = require("pid")
local funcs = require("functions")
-------------------------------------------------------------------------------
------------------------- GLOBAL SCOPE DEFINITIONS ----------------------------
-------------------------------------------------------------------------------
-- Channel control variables
local THROTTLE_CONTROL_OUTPUT_CHANNEL = 3
local CONTROL_OUTPUT_YAW = 4
local MAX_CHANNEL_OUTPUT = 1950
local MIN_CHANNEL_OUTPUT = 1050
local PWM_RANGE = 450
-- Vehicle type control
local VEHICLE_TYPE = param:get('SCR_USER5')
-- TRIM values
local PWM_TRIM_VALUE = tonumber(param:get('SERVO1_TRIM')) or 0
local RC1_TRIM_VALUE = param:get('RC1_TRIM')
local RC3_TRIM_VALUE = param:get('RC3_TRIM')
-- Signal smoothing logic
local last_manual_throttle = 0
local throttle_accel_rate_thresh = 0.5
local throttle_accel_rate = 0.5
local last_manual_steering = 0
local steering_accel_rate_thresh = 0.6
local steering_accel_rate = 0.6
-- Yaw alignment logic
local UPDATE_PERIOD_MS = 200
local UPDATE_DT = UPDATE_PERIOD_MS / 1000.0
local YAW_THRESH_RAD = math.rad(10)
local YAW_DEADBAND = 0.02
local YAW_ALIGN_TIMEOUT_MS = 15000
local YAW_ALIGN_MAX_STEPS = math.floor(YAW_ALIGN_TIMEOUT_MS / UPDATE_PERIOD_MS)
-- Params: p_gain, i_gain, d_gain, i_max, i_min, pid_max, pid_min
local yaw_pid = PID:new(0.5, 0.25, 0.2, 5, -5, 0.99, -0.99)
local yaw_align_active = false
local yaw_target_deg = nil
local yaw_target_rad = nil
local yaw_align_steps = 0

local last_nav_idx = nil

-- Severity for logging in GCS
MAV_SEVERITY = { EMERGENCY = 0, ALERT = 1, CRITICAL = 2, ERROR = 3, WARNING = 4, NOTICE = 5, INFO = 6, DEBUG = 7 }
-- Rover driving modes
DRIVING_MODES = { MANUAL = 0, STEERING = 3, HOLD = 4, AUTO = 10, GUIDED = 15 }
-- Mission states dictionary
MISSION_STATE = { IDLE = 0, RUNNING = 1, FINISHED = 2 }

-- Controle de impressão de param4
local last_logged_wp_index = -1

-------------------------------------------------------------------------------
-------------- FUNÇÕES DE LEITURA / DEBUG DE PARAM4 DA MISSÃO ----------------
-------------------------------------------------------------------------------
local function log_current_wp_param4()
  -- indice do waypoint de navegação atual
  local idx = mission:get_current_nav_index()
  if not idx then
    return
  end

  -- evita imprimir repetidamente o mesmo waypoint
  if idx == last_logged_wp_index then
    return
  end

  local item = mission:get_item(idx)
  if not item then
    return
  end

  local cmd = item:command()
  -- 16 = MAV_CMD_NAV_WAYPOINT
  if cmd == 16 then
    local p4 = item:param4() or 0
    gcs:send_text(
      MAV_SEVERITY.INFO,
      string.format("WP %d (NAV_WAYPOINT) param4 = %.3f", idx, p4)
    )
    last_logged_wp_index = idx
  end
end

-------------------------------------------------------------------------------
------------------------- LOW LEVEL ACTION FUNCTIONS --------------------------
-------------------------------------------------------------------------------
--[[
Control allocation
This function is responsible for the PWM signal allocation of the vehicle.
It receives the throttle (t) and steering (s) values and calculates the PWM values for the motors.
The function also takes into account the trim values for the PWM outputs.
-- @param t number - Throttle command from 0 (or more) to 1.0
-- @param s number - Steering command from -1.0 to 1.0
--]]
local function applyControlAllocation(t, s)
  -- We assign the PWM values to the motors, which are opposite in sign for each diagonal pair
  -- MOTOR SCHEMATIC IN ROVER FRAME
  -- 1 - 0     ^
  --   |       | Rover forward direction
  -- 2 - 3
  local pwm_aloc_l, pwm_aloc_r = funcs:allocateRightAndLeftPwmShare(t, s, PWM_RANGE)
  -- Limiting the output values to the PWM ranges
  local pwm_l = funcs:mapMaxMin(PWM_TRIM_VALUE + pwm_aloc_l, MIN_CHANNEL_OUTPUT, MAX_CHANNEL_OUTPUT)
  local pwm_r = funcs:mapMaxMin(PWM_TRIM_VALUE - pwm_aloc_r, MIN_CHANNEL_OUTPUT, MAX_CHANNEL_OUTPUT)
  -- Setting the PWM outputs based on the control allocation directions
  -- left for motors in the left side (1 and 2), right for the ones on the right side (0 and 3)
  SRV_Channels:set_output_pwm_chan_timeout(0, pwm_r, 300)
  SRV_Channels:set_output_pwm_chan_timeout(1, pwm_l, 300)
  SRV_Channels:set_output_pwm_chan_timeout(2, pwm_l, 300)
  SRV_Channels:set_output_pwm_chan_timeout(3, pwm_r, 300)
end

-------------------------------------------------------------------------------
--------------------- YAW CONTROL VIA MAV_CMD_NAV_WAYPOINT --------------------
-------------------------------------------------------------------------------
-- MAV_CMD_NAV_WAYPOINT (16) defines param4 as desired yaw angle (deg). If yaw
-- is not explicitly set, some GCS/flight-stacks may send NaN as "unset".
--
-- This block watches mission:get_current_nav_index(). When the index changes,
-- we treat the previous index as "reached" and, if it was a NAV_WAYPOINT with
-- a valid param4, we temporarily switch to STEERING mode and rotate in place
-- (throttle=0) until the heading error is within threshold, then go back to AUTO.
--
-- NOTE: This script assumes ahrs:get_yaw() returns yaw in radians.
-------------------------------------------------------------------------------

local function start_yaw_align(target_deg)
  yaw_target_deg = funcs:mapTo360(target_deg)
  yaw_target_rad = funcs:toRadians(yaw_target_deg)

  yaw_pid:resetInternalState()
  yaw_align_active = true
  yaw_align_steps = 0

  -- Stop the vehicle and take over yaw using STEERING mode
  applyControlAllocation(0, 0)
  vehicle:set_mode(DRIVING_MODES.STEERING)

  gcs:send_text(
    MAV_SEVERITY.INFO,
    string.format("Yaw align start (WP yaw=%.1f deg)", yaw_target_deg)
  )
end

local function step_yaw_align()
  -- If the pilot or failsafe switched modes, stop controlling
  if vehicle:get_mode() ~= DRIVING_MODES.STEERING then
    yaw_align_active = false
    yaw_pid:resetInternalState()
    return
  end

  yaw_align_steps = yaw_align_steps + 1

  -- Current yaw from AHRS (rad)
  local current_yaw_rad = ahrs:get_yaw()
  local err = funcs:yawErrorRad(current_yaw_rad, yaw_target_rad)

  -- Timeout safety
  if yaw_align_steps > YAW_ALIGN_MAX_STEPS then
    gcs:send_text(MAV_SEVERITY.WARNING, "Yaw align TIMEOUT -> back to AUTO")
    applyControlAllocation(0, 0)
    yaw_align_active = false
    yaw_pid:resetInternalState()
    vehicle:set_mode(DRIVING_MODES.AUTO)
    return
  end

  -- Done?
  if math.abs(err) <= YAW_THRESH_RAD then
    applyControlAllocation(0, 0)
    yaw_align_active = false
    yaw_pid:resetInternalState()
    gcs:send_text(
      MAV_SEVERITY.INFO,
      string.format("Yaw aligned (target %.1f deg) -> AUTO", yaw_target_deg)
    )
    vehicle:set_mode(DRIVING_MODES.AUTO)
    return
  end

  local pid_out = yaw_pid:compute(err, UPDATE_DT)
  if math.abs(pid_out) < YAW_DEADBAND then
    pid_out = 0
  end

  -- Rotate in place
  applyControlAllocation(0, pid_out)
end

local function maybe_trigger_yaw_align_on_reached_wp()
  local idx = mission:get_current_nav_index()
  if not idx then
    return
  end

  -- Initialize the index tracker
  if last_nav_idx == nil then
    last_nav_idx = idx
    return
  end

  -- When the nav index changes, the previous waypoint was reached
  if idx ~= last_nav_idx then
    local reached_idx = last_nav_idx
    last_nav_idx = idx

    local item = mission:get_item(reached_idx)
    if not item then
      return
    end

    -- Only handle NAV_WAYPOINT (16)
    if item:command() ~= 16 then
      return
    end

    local p4 = item:param4()
    if p4 == nil or funcs:isNan(p4) then
      return
    end

    -- param4 is yaw angle in degrees
    start_yaw_align(p4)
  end
end

--[[
Control the actions while not armed
--]]
local function notArmed()
  SRV_Channels:set_output_pwm_chan_timeout(0, PWM_TRIM_VALUE, 3000)
  SRV_Channels:set_output_pwm_chan_timeout(1, PWM_TRIM_VALUE, 3000)
  SRV_Channels:set_output_pwm_chan_timeout(2, PWM_TRIM_VALUE, 3000)
  SRV_Channels:set_output_pwm_chan_timeout(3, PWM_TRIM_VALUE, 3000)
end

--[[
Perform vehicle control in Manual mode
--]]
local function applyPWMManualMode()
  local rc3_pwm = rc:get_pwm(3)
  local rc1_pwm = rc:get_pwm(1)

  -- Compares the diff from the last manual signals to the maximum rate we are accepting
  -- Make the actual command be a rate from the last to the required command if necessary
  local raw_steering = (rc1_pwm - RC1_TRIM_VALUE) / 450
  local steering = funcs:applyAbsSmoothing(raw_steering, last_manual_steering, steering_accel_rate_thresh,
    steering_accel_rate)
  last_manual_steering = steering
  local raw_throttle = (RC3_TRIM_VALUE - rc3_pwm) / 450
  local throttle = funcs:applyAbsSmoothing(raw_throttle, last_manual_throttle, throttle_accel_rate_thresh,
    throttle_accel_rate)
  last_manual_throttle = throttle

  applyControlAllocation(throttle, steering)
end

--[[
Perform vehicle control in Steering mode
--]]
local function applyPWMSteeringMode()
  local rc1_pwm = rc:get_pwm(1)

  -- Compares the diff from the last steering signals to the maximum rate we are accepting
  -- Make the actual command be a rate from the last to the required command if necessary
  local raw_steering = (rc1_pwm - RC1_TRIM_VALUE) / 450
  local steering = funcs:applyAbsSmoothing(raw_steering, last_manual_steering, steering_accel_rate_thresh,
    steering_accel_rate)
  last_manual_steering = steering

  applyControlAllocation(0, steering)
end

-------------------------------------------------------------------------------
-------------------------------- MAIN LOOP ------------------------------------
-------------------------------------------------------------------------------
local function update()
  -- Safety check for vehicle type
  if not (VEHICLE_TYPE == 2) then
    gcs:send_text(MAV_SEVERITY.WARNING, string.format("Not ROVER, exiting LUA script."))
    return
  end
  -- Getting SCR_USER params to PID values
  local p, i, d = param:get('SCR_USER2') / 1000, param:get('SCR_USER3') / 1000, param:get('SCR_USER4') / 1000
  yaw_pid:setGains(p, i, d)

  -- Run not armed routine to guarantee trim values
  if not arming:is_armed() then
    notArmed()
    return update, 2000
  end

  -- If we are currently aligning yaw from a waypoint (param4), take over here
  if yaw_align_active then
    step_yaw_align()
    return update, 200
  end

  if vehicle:get_mode() == DRIVING_MODES.MANUAL then
    --[[
    Controlling in MANUAL MODE
    --]]
    applyPWMManualMode()
    return update, 200

  elseif vehicle:get_mode() == DRIVING_MODES.STEERING then
    --[[
    Controlling in STEERING MODE
    --]]
    applyPWMSteeringMode()
    return update, 200

  elseif vehicle:get_mode() == DRIVING_MODES.HOLD then
    --[[
    Controlling in HOLD MODE
    --]]
    -- Make the vehicle stop
    applyControlAllocation(0, 0)
    return update, 200

  elseif vehicle:get_mode() == DRIVING_MODES.AUTO then
    --[[
    Controlling in AUTO MODE
    --]]

    -- Logar param4 do waypoint atual (se for NAV_WAYPOINT)
    log_current_wp_param4()

    -- Controls end of mission
    local mission_state = mission:state()
    if mission_state == MISSION_STATE.FINISHED then
      applyControlAllocation(0, 0)
      vehicle:set_mode(DRIVING_MODES.MANUAL)
      return update, 200
    end

    -- Se o índice de navegação mudou, o waypoint anterior foi alcançado:
    -- se for NAV_WAYPOINT (16) e tiver param4 (yaw) válido, alinhar yaw e voltar ao AUTO
    maybe_trigger_yaw_align_on_reached_wp()
    if yaw_align_active then
      return update, 200
    end

    -- Acquiring throttle and steering from internal control output
    local throttle = tonumber(vehicle:get_control_output(THROTTLE_CONTROL_OUTPUT_CHANNEL))
    throttle = funcs:mapMaxMin(throttle, 0.1, 1.0)
    local steering = tonumber(vehicle:get_control_output(CONTROL_OUTPUT_YAW)) or 0

    -- Apply the control allocation finally
    applyControlAllocation(throttle, steering)
    return update, 200
  end
end

return update, 3000 -- run immediately before starting to reschedule
-------------------------------------------------------------------------------
