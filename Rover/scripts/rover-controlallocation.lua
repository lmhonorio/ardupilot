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
local YAW_THRESH_RAD = math.rad(param:get('SCR_USER1'))
local YAW_DEADBAND = 0.02
local YAW_ALIGN_TIMEOUT_MS = 15000
local REVERSE_ALT_MIN_DEG = 360
local REVERSE_ALT_MAX_DEG = 720
local REVERSE_ALT_OFFSET_DEG = 360
-- Params: p_gain, i_gain, d_gain, i_max, i_min, pid_max, pid_min
local yaw_pid = PID:new(0.5, 0.25, 0.2, 5, -5, 0.99, -0.99)
local yaw_target_rad = nil
local yaw_align_steps = 0

local last_nav_idx = nil
local reverse_to_next_wp = false
local WP_RADIUS = param:get('WP_RADIUS') or 2.0 -- meters

local radio_type = 0

-- Severity for logging in GCS
MAV_SEVERITY = { EMERGENCY = 0, ALERT = 1, CRITICAL = 2, ERROR = 3, WARNING = 4, NOTICE = 5, INFO = 6, DEBUG = 7 }
-- Rover driving modes
DRIVING_MODES = { MANUAL = 0, STEERING = 3, HOLD = 4, AUTO = 10, GUIDED = 15 }
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
-- @param t number - Throttle command from 0 (or more) to 1.0
-- @param s number - Steering command from -1.0 to 1.0
--]]
local function applyControlAllocation(t, s)
  local pwm_aloc_l, pwm_aloc_r = funcs:allocateRightAndLeftPwmShare(t, s, PWM_RANGE)
  -- We assign the PWM values to the motors, which are opposite in sign for each diagonal pair
  -- MOTOR SCHEMATIC IN ROVER FRAME
  -- 1 - 0     ^
  --   |       | Rover forward direction
  -- 2 - 3
  -- Setting the PWM outputs based on the control allocation directions
  -- left for motors in the left side (1 and 2), right for the ones on the right side (0 and 3)
  local pwm_l = funcs:mapMaxMin(PWM_TRIM_VALUE + pwm_aloc_l, MIN_CHANNEL_OUTPUT, MAX_CHANNEL_OUTPUT)
  local pwm_r = funcs:mapMaxMin(PWM_TRIM_VALUE - pwm_aloc_r, MIN_CHANNEL_OUTPUT, MAX_CHANNEL_OUTPUT)
  SRV_Channels:set_output_pwm_chan_timeout(0, pwm_r, 300)
  SRV_Channels:set_output_pwm_chan_timeout(1, pwm_l, 300)
  SRV_Channels:set_output_pwm_chan_timeout(2, pwm_l, 300)
  SRV_Channels:set_output_pwm_chan_timeout(3, pwm_r, 300)
end

--[[
Calculating the signals when driving reverse_to_next_wp
@param t number - Throttle command from 0 (or more) to 1.0
@param s number - Steering command from -1.0 to 1.0
@return number, number - The modified throttle and steering commands for reverse driving
--]]
local function calculateReverseOutputSignals(t, s)
  -- Controlling steering with PID using the waypoint target yaw
  local err = funcs:yawErrorRad(ahrs:get_yaw(), yaw_target_rad)
  local s_out = yaw_pid:compute(err, UPDATE_DT)
  gcs:send_text(MAV_SEVERITY.DEBUG, string.format("Yaw error: %.2f deg, PID output: %.2f", math.deg(err), s_out))
  if math.abs(s_out) < YAW_DEADBAND then
    s_out = 0
  end

  -- Obtain the target waypoint coordinates and current vehicle position to calculate the distance in meters using the haversine functions
  local idx = mission:get_current_nav_index()
  if not idx then
    return -t, s_out
  end
  local target_wp = mission:get_item(idx)
  if not target_wp then
    return -t, s_out
  end
  local target_lat = target_wp:x() / 1e7
  local target_lon = target_wp:y() / 1e7
  local current_lat = ahrs:get_location():lat() / 1e7
  local current_lon = ahrs:get_location():lng() / 1e7
  local distance_to_wp = funcs:haversineDistance(current_lat, current_lon, target_lat, target_lon)
  -- If the distance is bigger than the waypoint radius, set a constant throttle to 0.3
  if distance_to_wp > 3 * WP_RADIUS then
    return -0.3, s_out
  end

  return -t, s_out
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
  yaw_pid:resetInternalState()
  reverse_to_next_wp = false
end

--[[
Decode yaw and reverse direction from waypoint z (altitude) field
Encoding:
  -1: pass-through waypoint
  0..360: align yaw and drive forward on next leg
  360..720: align yaw to (z-360) and drive reverse on next leg
-- @param angle_from_alt number
-- @return number|nil, bool, bool
--]]
local function decodeYawAndDirectionFromWaypointZ(angle_from_alt)
  if angle_from_alt == nil or funcs:isNan(angle_from_alt) then
    return nil, false, false
  end
  if angle_from_alt == -1 then
    return nil, false, true
  end

  local reverse_leg = angle_from_alt >= REVERSE_ALT_MIN_DEG and angle_from_alt <= REVERSE_ALT_MAX_DEG
  local yaw_deg = angle_from_alt
  if reverse_leg then
    yaw_deg = yaw_deg - REVERSE_ALT_OFFSET_DEG
  end

  return funcs:mapTo360(yaw_deg), reverse_leg, false
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
    reverse_to_next_wp = false
    if not item then
      resetYawControlState()
      return false
    end

    -- Only handle NAV_WAYPOINT (16) with valid param4 (yaw)
    if item:command() ~= 16 then
      resetYawControlState()
      return false
    end

    local yaw_target_deg, reverse_leg, is_pass_through = decodeYawAndDirectionFromWaypointZ(item:z())

    -- If angle == -1, treat as pass-through waypoint: do NOT switch modes
    if is_pass_through then
      -- clear any previous target just in case
      resetYawControlState()
      return false
    end

    reverse_to_next_wp = reverse_leg
    if yaw_target_deg == nil then
      resetYawControlState()
      return false
    end
    yaw_target_rad = math.rad(yaw_target_deg)

    -- Reset PID state and start alignment
    yaw_pid:resetInternalState()
    yaw_align_steps = 0
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
  reverse_to_next_wp = false
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
    yaw_pid:resetInternalState()
    return
  end

  -- Timeout safety
  yaw_align_steps = yaw_align_steps + 1
  local yaw_align_max_steps = math.floor(YAW_ALIGN_TIMEOUT_MS / UPDATE_PERIOD_MS)
  if reverse_to_next_wp then
    yaw_align_max_steps = yaw_align_max_steps * 2 -- allow more time for reverse maneuvers
  end
  if yaw_align_steps > yaw_align_max_steps then
    applyControlAllocation(0, 0)
    yaw_pid:resetInternalState()
    vehicle:set_mode(DRIVING_MODES.AUTO)
    return
  end

  -- Current yaw from AHRS (rad) and error to target
  local err = funcs:yawErrorRad(ahrs:get_yaw(), yaw_target_rad)

  -- Check if we reached the target yaw
  if math.abs(err) <= YAW_THRESH_RAD then
    applyControlAllocation(0, 0)
    yaw_pid:resetInternalState()
    -- Set HOLD mode so the vehicle stops before going back to AUTO
    vehicle:set_mode(DRIVING_MODES.HOLD)
    return
  end

  -- Rotate in place with pid output
  local pid_out = yaw_pid:compute(err, UPDATE_DT)
  if math.abs(pid_out) < YAW_DEADBAND then
    pid_out = 0
  end
  applyControlAllocation(0, pid_out)
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

  -- When starting script in the middle of a mission, infer direction from previous waypoint
  if idx and last_nav_idx == nil and idx > 0 then
    local previous_item = mission:get_item(idx - 1)
    if previous_item and previous_item:command() == 16 then
      local _, reverse_leg, _ = decodeYawAndDirectionFromWaypointZ(previous_item:z())
      reverse_to_next_wp = reverse_leg
    else
      reverse_to_next_wp = false
    end
  end

  -- Controls end of mission
  local mission_state = mission:state()
  if mission_state == MISSION_STATE.FINISHED then
    applyControlAllocation(0, 0)
    reverse_to_next_wp = false
    vehicle:set_mode(DRIVING_MODES.MANUAL)
    return update, 200
  end

  -- If we reached a waypoint, check if we need to align yaw from param4 with a valid value
  if triggerYawControlOnReachedWaypoint() then
    return update, 200
  end

  -- Acquiring throttle and steering from internal control output
  local throttle = tonumber(vehicle:get_control_output(THROTTLE_CONTROL_OUTPUT_CHANNEL)) or 0
  throttle = funcs:mapMaxMin(math.abs(throttle), 0.1, 1.0)
  local steering = tonumber(vehicle:get_control_output(CONTROL_OUTPUT_YAW)) or 0
  -- Reverse signals in case the waypoint tells us to drive backwards on the next leg
  if reverse_to_next_wp then
    throttle, steering = calculateReverseOutputSignals(throttle, steering)
  end
  applyControlAllocation(throttle, steering)
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
  local p, i, d = param:get('SCR_USER2') / 1000, param:get('SCR_USER3') / 1000, param:get('SCR_USER4') / 1000
  yaw_pid:setGains(p, i, d)

  -- Getting radio type
  radio_type = param:get('RC3_REVERSED') or 0

  -- Run not armed routine to guarantee trim values
  if not arming:is_armed() then
    resetYawControlState()
    last_nav_idx = nil
    notArmed()
    return update, 2000
  end

  if vehicle:get_mode() == DRIVING_MODES.MANUAL then
    applyPWMManualMode()
    return update, 200
  elseif vehicle:get_mode() == DRIVING_MODES.STEERING then
    -- We use steering mode for yaw alignment
    applyPWMSteeringMode()
    return update, 200
  elseif vehicle:get_mode() == DRIVING_MODES.HOLD then
    -- Make the vehicle stop
    applyControlAllocation(0, 0)
    return update, 200
  elseif vehicle:get_mode() == DRIVING_MODES.AUTO then
    applyPWMAutoMode()
    return update, 200
  end
end

return update, 3000 -- run immediately before starting to reschedule
-------------------------------------------------------------------------------
