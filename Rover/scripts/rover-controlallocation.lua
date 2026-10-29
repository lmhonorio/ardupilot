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
package.path = package.path .. ';/scripts/modules/?.lua'
local PID = require("pid")
local funcs = require("functions")
local Autotune = require("autotune")
-------------------------------------------------------------------------------
------------------------- GLOBAL SCOPE DEFINITIONS ----------------------------
-------------------------------------------------------------------------------
-- Channel control variables
local THROTTLE_CONTROL_OUTPUT_CHANNEL = 3
local CONTROL_OUTPUT_YAW = 4
local THROTTLE_PARAM_TABLE_KEY = 17
local THROTTLE_PARAM_TABLE_PREFIX = 'THRO_'
assert(param:add_table(THROTTLE_PARAM_TABLE_KEY, THROTTLE_PARAM_TABLE_PREFIX, 1), 'could not add Lua throttle parameter table')
--[[
  // @Param: THRO_MAX_LUA
  // @DisplayName: Lua maximum motor output
  // @Description: Maximum motor PWM range as a percentage of the 1000-2000 us range around the 1500 us trim
  // @Units: %
  // @Range: 0 100
  // @Increment: 1
  // @User: Standard
--]]
assert(param:add_param(THROTTLE_PARAM_TABLE_KEY, 1, 'MAX_LUA', 90), 'could not add THRO_MAX_LUA parameter')
local throttle_max_lua_param = Parameter()
assert(throttle_max_lua_param:init('THRO_MAX_LUA'), 'could not bind THRO_MAX_LUA parameter')
-- Vehicle type control
local VEHICLE_TYPE = param:get('SCR_USER5')
-- TRIM values
local PWM_TRIM_VALUE = 1500
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
local YAW_THRESH_RAD = math.rad(2)
local YAW_RATE_THRESH_RAD_S = math.rad(2)
local YAW_SETTLE_STEPS = 3 -- require 600 ms continuously within angle and rate limits
local YAW_DEADBAND = 0.02
local YAW_ALIGN_TIMEOUT_MS = 15000
local YAW_ALIGN_LOG_INTERVAL_MS = 1000
local last_yaw_align_log_ms = 0
-- Params: p_gain, i_gain, d_gain, i_max, pid_max
local steering_steady_pid = PID:new(3.5, 8, 0, 1, 0.95)
local yaw_target_rad = nil
local yaw_align_steps = 0
local yaw_settle_steps = 0

local last_nav_idx = nil

local radio_type = 0

-- Tuning selector parameter (SCR_USER1):
--   0 = normal operation
--   1 = Pixhawk PID tune (Rover QuickTune) - starts while armed
--   2 = yaw PI autotune (relay)            - starts while armed
-- After a successful tune the selector returns to 0 automatically.
-- Yaw autotune is selected only through SCR_USER1.
-- Results are saved to SCR_USER2 (P) and SCR_USER3 (I).
local TUNE_SELECT_PARAM = 'SCR_USER1'
local TUNE_MODE = { NORMAL = 0, PIXHAWK = 1, YAW = 2 }
local AUTOTUNE_PERIOD_MS = UPDATE_PERIOD_MS

-- Yaw autotune parameters
local YAW_TUNE_PARAM_TABLE_KEY = 16
assert(param:add_table(YAW_TUNE_PARAM_TABLE_KEY, 'YAWT_', 2), 'could not add yaw autotune parameter table')
assert(param:add_param(YAW_TUNE_PARAM_TABLE_KEY, 1, 'MAX_ANG', 45), 'could not add yaw autotune max angle parameter')
--[[
  // @Param: YAWT_MAX_ANG
  // @DisplayName: Yaw autotune maximum angle
  // @Description: Abort yaw autotune if the yaw error exceeds this angle
  // @Units: deg
  // @Range: 5 90
  // @User: Standard
--]]
local yaw_max_angle_param = Parameter()
assert(yaw_max_angle_param:init('YAWT_MAX_ANG'), 'could not bind YAWT_MAX_ANG parameter')

--[[
  // @Param: YAWT_MAX_OUT
  // @DisplayName: Yaw autotune maximum motor output
  // @Description: Maximum steering command applied by yaw autotune; lower values reduce wheel spin speed
  // @Units: %
  // @Range: 5 45
  // @User: Standard
--]]
assert(param:add_param(YAW_TUNE_PARAM_TABLE_KEY, 2, 'MAX_OUT', 20), 'could not add yaw autotune max output parameter')
local yaw_max_out_param = Parameter()
assert(yaw_max_out_param:init('YAWT_MAX_OUT'), 'could not bind YAWT_MAX_OUT parameter')

local autotune = Autotune:new({ relay = 0.20 })
local autotune_target_rad = nil
local autotune_done = false
local yaw_tune_last_progress = 0
local yaw_tune_last_log_s = 0
local selector_armed_reported = false

-- Load tuned gains if present (0 means "not tuned yet", keep defaults)
local saved_p, saved_i = param:get('SCR_USER2') or 0, param:get('SCR_USER3') or 0
if saved_p > 0 and saved_i > 0 then
  steering_steady_pid:setGains(saved_p, saved_i, 0)
end

-- Severity for logging in GCS
MAV_SEVERITY = { EMERGENCY = 0, ALERT = 1, CRITICAL = 2, ERROR = 3, WARNING = 4, NOTICE = 5, INFO = 6, DEBUG = 7 }
-- Rover driving modes
DRIVING_MODES = { MANUAL = 0, ACRO = 1, STEERING = 3, HOLD = 4, LOITER = 5, SIMPLE = 7, CIRCLE = 9, AUTO = 10, RTL = 11, SMART_RTL = 12, GUIDED = 15 }
-- Mission states dictionary
MISSION_STATE = { IDLE = 0, RUNNING = 1, FINISHED = 2 }

-------------------------------------------------------------------------------
------------------------- LOW LEVEL ACTION FUNCTIONS --------------------------
-------------------------------------------------------------------------------
--[[
Control allocation
This function is responsible for the PWM signal allocation of the vehicle.
It receives the throttle (t) and steering (s) values and calculates the PWM values for the motors.
The function also takes into account the trim values for the PWM outputs.
-- @param t number - Throttle command from -1.0 to 1.0
-- @param s number - Steering command from -1.0 to 1.0
--]]
local function applyControlAllocation(t, s)
  -- We assign the PWM values to the motors, which are opposite in sign for each diagonal pair
  -- MOTOR SCHEMATIC IN ROVER FRAME
  -- 1 - 0     ^
  --   |       | Rover forward direction
  -- 2 - 3
  -- Setting the PWM outputs based on the control allocation directions
  -- left for motors in the left side (1 and 2), right for the ones on the right side (0 and 3)
  local throttle_limit_pct = math.max(0, math.min(100, throttle_max_lua_param:get()))
  local pwm_range = 5 * throttle_limit_pct
  local pwm_aloc_l, pwm_aloc_r = funcs:allocateRightAndLeftPwmShare(t, s, pwm_range)
  local pwm_min = PWM_TRIM_VALUE - pwm_range
  local pwm_max = PWM_TRIM_VALUE + pwm_range
  local pwm_l = math.floor(funcs:mapMaxMin(PWM_TRIM_VALUE + pwm_aloc_l, pwm_min, pwm_max))
  local pwm_r = math.floor(funcs:mapMaxMin(PWM_TRIM_VALUE - pwm_aloc_r, pwm_min, pwm_max))
  SRV_Channels:set_output_pwm_chan_timeout(0, pwm_r, 300)
  SRV_Channels:set_output_pwm_chan_timeout(1, pwm_l, 300)
  SRV_Channels:set_output_pwm_chan_timeout(2, pwm_l, 300)
  SRV_Channels:set_output_pwm_chan_timeout(3, pwm_r, 300)
end

-------------------------------------------------------------------------------
--------------------- YAW CONTROL VIA MAV_CMD_NAV_WAYPOINT --------------------
-------------------------------------------------------------------------------
--[[
Reset the yaw control state
--]]
local function resetYawControlState()
  yaw_target_rad = nil
  yaw_align_steps = 0
  yaw_settle_steps = 0
  steering_steady_pid:resetInternalState()
end

--[[
Decode yaw from waypoint z (altitude) field
Encoding:
  -1: pass-through waypoint
  Any other value: align yaw to the angle informed in the waypoint altitude
-- @param angle_from_alt number
-- @return number|nil, bool
--]]
local function decodeYawFromWaypointZ(angle_from_alt)
  if angle_from_alt == nil or funcs:isNan(angle_from_alt) then
    return nil, false
  end

  if angle_from_alt == -1 then
    return nil, true
  end

  return funcs:mapTo360(angle_from_alt), false
end

--[[
Check if a waypoint was reached and trigger yaw control if param4 is valid
-- @return bool - true if yaw control was triggered
--]]
local function triggerYawControlOnReachedWaypoint()
  local idx = mission:get_current_nav_index()
  if not idx then
    return false
  end

  -- Initialize the index tracker
  if last_nav_idx == nil then
    last_nav_idx = idx
    return false
  end

  -- When the nav index changes, the previous waypoint was reached
  if idx ~= last_nav_idx then
    local reached_idx = last_nav_idx
    last_nav_idx = idx
    local item = mission:get_item(reached_idx)
    if not item then
      resetYawControlState()
      return false
    end

    -- Only handle NAV_WAYPOINT (16) with valid param4 (yaw)
    if item:command() ~= 16 then
      resetYawControlState()
      return false
    end

    local yaw_target_deg, is_pass_through = decodeYawFromWaypointZ(item:z())

    -- If angle == -1, treat as pass-through waypoint: do NOT switch modes
    -- This value must be used in the waypoint before a DO_SET_REVERSE transition
    -- so the mission remains in AUTO while ArduRover changes the driving direction.
    if is_pass_through then
      -- clear any previous target just in case
      resetYawControlState()
      return false
    end

    if yaw_target_deg == nil then
      resetYawControlState()
      return false
    end

    yaw_target_rad = math.rad(yaw_target_deg)

    -- Reset PID state and start alignment
    steering_steady_pid:resetInternalState()
    yaw_align_steps = 0
    yaw_settle_steps = 0
    -- Stop the vehicle and take over yaw using STEERING mode
    applyControlAllocation(0, 0)
    vehicle:set_mode(DRIVING_MODES.STEERING)
    return true
  end
  return false
end

-------------------------------------------------------------------------------
------------------------- HIGH LEVEL CONTROL FUNCTIONS ------------------------
-------------------------------------------------------------------------------
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
  local raw_throttle = 0

  -- Compares the diff from the last manual signals to the maximum rate we are accepting
  -- Make the actual command be a rate from the last to the required command if necessary
  local raw_steering = (rc1_pwm - RC1_TRIM_VALUE) / 450
  local steering = funcs:applyAbsSmoothing(raw_steering, last_manual_steering, steering_accel_rate_thresh,
    steering_accel_rate)
  last_manual_steering = steering
  if radio_type == 1 then
    raw_throttle = (rc3_pwm - RC3_TRIM_VALUE) / 450
  else
    raw_throttle = (RC3_TRIM_VALUE - rc3_pwm) / 450
  end
  local throttle = funcs:applyAbsSmoothing(raw_throttle, last_manual_throttle, throttle_accel_rate_thresh,
    throttle_accel_rate)
  last_manual_throttle = throttle

  applyControlAllocation(throttle, steering)
end

--[[
Perform vehicle control in Steering mode
--]]
local function applyPWMSteeringMode()
  -- If the pilot or failsafe switched modes, stop pursuing yaw alignment
  if vehicle:get_mode() ~= DRIVING_MODES.STEERING then
    steering_steady_pid:resetInternalState()
    yaw_settle_steps = 0
    return
  end

  -- Timeout safety
  yaw_align_steps = yaw_align_steps + 1
  local yaw_align_max_steps = math.floor(YAW_ALIGN_TIMEOUT_MS / UPDATE_PERIOD_MS)
  if yaw_align_steps > yaw_align_max_steps then
    applyControlAllocation(0, 0)
    steering_steady_pid:resetInternalState()
    yaw_settle_steps = 0
    vehicle:set_mode(DRIVING_MODES.AUTO)
    return
  end

  -- Current yaw from AHRS (rad) and error to target
  local current_yaw_rad = ahrs:get_yaw()
  local err = funcs:yawErrorRad(current_yaw_rad, yaw_target_rad)
  local now_ms = millis():toint()
  if now_ms - last_yaw_align_log_ms >= YAW_ALIGN_LOG_INTERVAL_MS then
    last_yaw_align_log_ms = now_ms
    gcs:send_text(MAV_SEVERITY.INFO, string.format(
      "YawAlign target=%.1f current=%.1f error=%.1f",
      math.deg(yaw_target_rad), math.deg(current_yaw_rad), math.deg(err)))
  end

  -- Require both heading and yaw rate to settle before stopping. This avoids
  -- switching to HOLD while the rover is still rotating through the target.
  local yaw_rate = ahrs:get_gyro():z()
  if math.abs(err) <= YAW_THRESH_RAD and math.abs(yaw_rate) <= YAW_RATE_THRESH_RAD_S then
    yaw_settle_steps = yaw_settle_steps + 1
  else
    yaw_settle_steps = 0
  end

  if yaw_settle_steps >= YAW_SETTLE_STEPS then
    applyControlAllocation(0, 0)
    steering_steady_pid:resetInternalState()
    yaw_settle_steps = 0
    -- Set HOLD mode so the vehicle stops for the external ROS photo routine
    vehicle:set_mode(DRIVING_MODES.HOLD)
    return
  end

  -- Rotate in place with pid output
  local s_out = steering_steady_pid:compute(err, UPDATE_DT)
  if math.abs(s_out) < YAW_DEADBAND then
    s_out = 0
  end
  applyControlAllocation(0, s_out)
end

--[[
Perform vehicle control in Auto mode
--]]
local function applyPWMAutoMode()
  local idx = mission:get_current_nav_index()

  -- Detect mission restart / rewind: current index went backwards
  if idx and last_nav_idx and idx < last_nav_idx then
    resetYawControlState()
    last_nav_idx = nil
  end

  -- Controls end of mission
  local mission_state = mission:state()
  if mission_state == MISSION_STATE.FINISHED then
    applyControlAllocation(0, 0)
    vehicle:set_mode(DRIVING_MODES.MANUAL)
    return
  end

  -- If we reached a waypoint, check if we need to align yaw from param4 with a valid value
  if triggerYawControlOnReachedWaypoint() then
    return
  end

  -- Acquiring throttle and steering from internal control output
  -- The throttle sign is preserved so MAV_CMD_DO_SET_REVERSE can be handled natively by ArduRover.
  local throttle = tonumber(vehicle:get_control_output(THROTTLE_CONTROL_OUTPUT_CHANNEL)) or 0
  local steering = tonumber(vehicle:get_control_output(CONTROL_OUTPUT_YAW)) or 0
  applyControlAllocation(throttle, steering)
end

--[[
Pass the ArduRover internal controllers' outputs straight to the allocation
(used by CIRCLE/ACRO/RTL, e.g. during Rover QuickTune)
--]]
local function applyPWMInternalControl()
  local throttle = tonumber(vehicle:get_control_output(THROTTLE_CONTROL_OUTPUT_CHANNEL)) or 0
  local steering = tonumber(vehicle:get_control_output(CONTROL_OUTPUT_YAW)) or 0
  applyControlAllocation(throttle, steering)
end

--[[
Yaw PI autotune in HOLD mode.
Switch high starts the relay test around the current yaw; switch low aborts/rearms.
-- @return bool - true while the autotune owns the outputs
--]]
local function runYawAutotune()
  if autotune_done then return false end
  if autotune.status == "idle" then
    autotune_target_rad = ahrs:get_yaw()
    autotune.relay = math.max(0.05, math.min(0.45, yaw_max_out_param:get() / 100))
    autotune:start()
    yaw_tune_last_progress = 0
    yaw_tune_last_log_s = -1
    gcs:send_text(MAV_SEVERITY.NOTICE, string.format(
      "YawTune: started; target %.1f deg, max error %.1f deg, max output %.0f%%",
      math.deg(autotune_target_rad), yaw_max_angle_param:get(), autotune.relay * 100))
  end
  local err = funcs:yawErrorRad(ahrs:get_yaw(), autotune_target_rad)
  local yaw_rate = ahrs:get_gyro():z()
  autotune.max_amp_rad = math.rad(yaw_max_angle_param:get())
  local s_out, status = autotune:step(err, AUTOTUNE_PERIOD_MS / 1000.0)
  applyControlAllocation(0, s_out)
  if autotune.cycle > yaw_tune_last_progress or autotune.t - yaw_tune_last_log_s >= 2 then
    yaw_tune_last_progress = autotune.cycle
    yaw_tune_last_log_s = autotune.t
    gcs:send_text(MAV_SEVERITY.INFO, string.format(
      "YawTune: t %.0fs cycle %d/%d err %.1fdeg rate %.1fdeg/s out %.2f",
      autotune.t, autotune.cycle, autotune.skip_cycles + autotune.cycles,
      math.deg(err), math.deg(yaw_rate), s_out))
  end
  if status == "done" then
    local r = autotune:result()
    param:set_and_save('SCR_USER2', r.P)
    param:set_and_save('SCR_USER3', r.I)
    steering_steady_pid:setGains(r.P, r.I, 0)
    steering_steady_pid:resetInternalState()
    gcs:send_text(MAV_SEVERITY.NOTICE, string.format("YawTune: Ku=%.2f Pu=%.2fs a=%.1fdeg", r.Ku, r.Pu, math.deg(r.amp)))
    gcs:send_text(MAV_SEVERITY.NOTICE, string.format("YawTune: saved P=%.3f I=%.3f", r.P, r.I))
    gcs:send_text(MAV_SEVERITY.NOTICE, "YawTune: finished successfully; steering output stopped")
    if param:get(TUNE_SELECT_PARAM) == TUNE_MODE.YAW then
      param:set_and_save(TUNE_SELECT_PARAM, TUNE_MODE.NORMAL)
      gcs:send_text(MAV_SEVERITY.NOTICE, "YawTune: SCR_USER1 set to 0 (normal)")
    end
    autotune_done = true
    applyControlAllocation(0, 0)
  elseif status == "fail" then
    gcs:send_text(MAV_SEVERITY.ERROR, "YawTune: failed - " .. autotune:result().msg)
    gcs:send_text(MAV_SEVERITY.WARNING, "YawTune: steering output stopped; selector remains available for retry")
    autotune_done = true
    applyControlAllocation(0, 0)
  end
  return true
end

-------------------------------------------------------------------------------
-------------------------------- MAIN LOOP ------------------------------------
-------------------------------------------------------------------------------
--[[
Main update function for the rover control allocation script
--]]
local function update()
  -- Safety check for vehicle type
  if not (VEHICLE_TYPE == 2) then
    gcs:send_text(MAV_SEVERITY.WARNING, string.format("Not ROVER, exiting LUA script."))
    return
  end
  -- Getting SCR_USER params to PID values
  -- local p, i, d = param:get('SCR_USER2') / 1000, param:get('SCR_USER3') / 1000, param:get('SCR_USER4') / 1000
  -- steering_steady_pid:setGains(p, i, d)

  -- Getting radio type
  radio_type = param:get('RC3_REVERSED') or 0

  -- Run not armed routine to guarantee trim values
  if not arming:is_armed() then
    if autotune.status == "running" then
      gcs:send_text(MAV_SEVERITY.WARNING, "YawTune: aborted (disarmed)")
    end
    selector_armed_reported = false
    resetYawControlState()
    last_nav_idx = nil
    autotune:reset()
    autotune_done = false
    notArmed()
    return update, 2000
  end

  local vehicle_mode = vehicle:get_mode()
  local selector = param:get(TUNE_SELECT_PARAM)
  if not selector_armed_reported then
    if selector == TUNE_MODE.YAW then
      gcs:send_text(MAV_SEVERITY.INFO, string.format("YawTune: SCR_USER1=2; starting while armed (mode %d)", vehicle_mode))
    elseif selector == TUNE_MODE.PIXHAWK then
      gcs:send_text(MAV_SEVERITY.INFO, string.format("RTun: SCR_USER1=1; QuickTune enabled while armed (mode %d)", vehicle_mode))
    end
    selector_armed_reported = selector == TUNE_MODE.YAW or selector == TUNE_MODE.PIXHAWK
  end
  if selector ~= TUNE_MODE.YAW and (autotune.status ~= "idle" or autotune_done) then
    if autotune.status == "running" then
      gcs:send_text(MAV_SEVERITY.WARNING, "YawTune: aborted (SCR_USER1 changed)")
    end
    autotune:reset()
    autotune_done = false
  elseif selector == TUNE_MODE.YAW then
    runYawAutotune()
    if autotune.status == "fail" or autotune_done then applyControlAllocation(0, 0) end
    return update, AUTOTUNE_PERIOD_MS
  end
  if vehicle_mode == DRIVING_MODES.MANUAL then
    applyPWMManualMode()
    return update, 200
  elseif vehicle_mode == DRIVING_MODES.STEERING then
    -- We use steering mode for yaw alignment
    applyPWMSteeringMode()
    return update, 200
  elseif vehicle_mode == DRIVING_MODES.AUTO then
    applyPWMAutoMode()
    return update, 200
  elseif vehicle_mode == DRIVING_MODES.SIMPLE then
    applyPWMInternalControl()
    return update, 200
  elseif vehicle_mode == DRIVING_MODES.CIRCLE or vehicle_mode == DRIVING_MODES.ACRO
      or vehicle_mode == DRIVING_MODES.RTL then
    applyPWMInternalControl()
    return update, 200
  elseif vehicle_mode == DRIVING_MODES.HOLD or vehicle_mode == DRIVING_MODES.GUIDED then
    -- Make the vehicle stop
    applyControlAllocation(0, 0)
    return update, 200
  end
  -- Unhandled mode: stop safely but keep the script alive
  applyControlAllocation(0, 0)
  return update, 200
end

gcs:send_text(MAV_SEVERITY.NOTICE, string.format(
  "Control allocation loaded; vehicle type=%.1f, SCR_USER1=%.1f, YAWT_MAX_ANG=%.1f deg, YAWT_MAX_OUT=%.0f%%",
  VEHICLE_TYPE or -1, param:get(TUNE_SELECT_PARAM) or -1, yaw_max_angle_param:get(), yaw_max_out_param:get()))
return update, 3000 -- run immediately before starting to reschedule
-------------------------------------------------------------------------------
