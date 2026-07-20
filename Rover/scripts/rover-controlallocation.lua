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
local YAW_THRESH_RAD = math.rad(2)
local YAW_DEADBAND = 0.02
local YAW_ALIGN_TIMEOUT_MS = 15000
local REVERSE_ALT_MIN_DEG = 360
local REVERSE_ALT_MAX_DEG = 720
local REVERSE_ALT_OFFSET_DEG = 360
-- Params: p_gain, i_gain, d_gain, i_max, pid_max
local steering_steady_pid = PID:new(3.5, 8, 0, 1, 0.95)
local steering_reverse_pid = PID:new(8, 1, 0, 1, 0.95)
local yaw_target_rad = nil
local yaw_align_steps = 0

local last_nav_idx = nil
local last_reverse_nav_idx = nil
local reverse_to_next_wp = false
local reverse_warning_reason = nil
local reverse_status_steps = 0
local reverse_armed_nav_idx = nil
local WP_RADIUS = param:get('WP_RADIUS') or 2.0 -- meters
local REVERSE_THROTTLE_FAR = -0.30
local REVERSE_THROTTLE_MID = -0.15
local REVERSE_THROTTLE_NEAR = -0.08

local radio_type = 0

-- Severity for logging in GCS
MAV_SEVERITY = { EMERGENCY = 0, ALERT = 1, CRITICAL = 2, ERROR = 3, WARNING = 4, NOTICE = 5, INFO = 6, DEBUG = 7 }
-- Rover driving modes
DRIVING_MODES = { MANUAL = 0, ACRO = 1, STEERING = 3, HOLD = 4, AUTO = 10, RTL = 11, GUIDED = 15 }
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
  -- We assign the PWM values to the motors, which are opposite in sign for each diagonal pair
  -- MOTOR SCHEMATIC IN ROVER FRAME
  -- 1 - 0     ^
  --   |       | Rover forward direction
  -- 2 - 3
  -- Setting the PWM outputs based on the control allocation directions
  -- left for motors in the left side (1 and 2), right for the ones on the right side (0 and 3)
  local pwm_aloc_l, pwm_aloc_r = funcs:allocateRightAndLeftPwmShare(t, s, PWM_RANGE)
  local pwm_l = funcs:mapMaxMin(PWM_TRIM_VALUE + pwm_aloc_l, MIN_CHANNEL_OUTPUT, MAX_CHANNEL_OUTPUT)
  local pwm_r = funcs:mapMaxMin(PWM_TRIM_VALUE - pwm_aloc_r, MIN_CHANNEL_OUTPUT, MAX_CHANNEL_OUTPUT)
  SRV_Channels:set_output_pwm_chan_timeout(0, pwm_r, 300)
  SRV_Channels:set_output_pwm_chan_timeout(1, pwm_l, 300)
  SRV_Channels:set_output_pwm_chan_timeout(2, pwm_l, 300)
  SRV_Channels:set_output_pwm_chan_timeout(3, pwm_r, 300)
end

--[[
Send one GCS warning per reverse navigation fault until valid data returns.
-- @param reason string
-- @param message string
--]]
local function sendReverseWarningOnce(reason, message)
  if reverse_warning_reason ~= reason then
    gcs:send_text(MAV_SEVERITY.WARNING, message)
    reverse_warning_reason = reason
  end
end

--[[
Calculating the signals when driving reverse_to_next_wp
@param t number - Throttle command from 0 (or more) to 1.0
@param s number - Steering command from -1.0 to 1.0
@return number, number - The modified throttle and steering commands for reverse driving
--]]
local function calculateReverseOutputSignals(t, s)
  local idx = mission:get_current_nav_index()
  if not idx then
    steering_reverse_pid:resetInternalState()
    last_reverse_nav_idx = nil
    sendReverseWarningOnce("missing_nav_index", "Reverse nav: no current waypoint index.")
    return 0, 0
  end

  if last_reverse_nav_idx ~= idx then
    steering_reverse_pid:resetInternalState()
    last_reverse_nav_idx = idx
  end

  local target_wp = mission:get_item(idx)
  if not target_wp then
    steering_reverse_pid:resetInternalState()
    sendReverseWarningOnce("missing_waypoint", "Reverse nav: no current waypoint item.")
    return 0, 0
  end

  local current_location = ahrs:get_location()
  if not current_location then
    steering_reverse_pid:resetInternalState()
    sendReverseWarningOnce("missing_location", "Reverse nav: no valid vehicle location.")
    return 0, 0
  end

  local current_yaw = ahrs:get_yaw()
  if current_yaw == nil then
    steering_reverse_pid:resetInternalState()
    sendReverseWarningOnce("missing_yaw", "Reverse nav: no valid vehicle yaw.")
    return 0, 0
  end

  local target_x = target_wp:x()
  local target_y = target_wp:y()
  if target_x == nil or target_y == nil then
    steering_reverse_pid:resetInternalState()
    sendReverseWarningOnce("invalid_waypoint", "Reverse nav: invalid waypoint coordinates.")
    return 0, 0
  end

  local target_lat = target_x / 1e7
  local target_lon = target_y / 1e7
  local current_lat_raw = current_location:lat()
  local current_lon_raw = current_location:lng()
  if current_lat_raw == nil or current_lon_raw == nil then
    steering_reverse_pid:resetInternalState()
    sendReverseWarningOnce("invalid_location", "Reverse nav: invalid vehicle coordinates.")
    return 0, 0
  end

  local current_lat = current_lat_raw / 1e7
  local current_lon = current_lon_raw / 1e7

  reverse_warning_reason = nil

  local distance_to_wp = funcs:haversineDistance(current_lat, current_lon, target_lat, target_lon)
  local reverse_throttle = REVERSE_THROTTLE_NEAR
  if distance_to_wp > 3 * WP_RADIUS then
    reverse_throttle = REVERSE_THROTTLE_FAR
  elseif distance_to_wp > WP_RADIUS then
    reverse_throttle = REVERSE_THROTTLE_MID
  end

  -- The bearing points from the rover position to the waypoint.
  local bearing_to_wp = funcs:bearingBetweenCoordinates(current_lat, current_lon, target_lat, target_lon)
  -- While reversing, the vehicle body should face away from that bearing so its rear moves toward the waypoint.
  local reverse_target_yaw = funcs:wrapToPi(bearing_to_wp + math.pi)
  -- The PID uses the body yaw error, not the bearing itself, to steer continuously during the reverse leg.
  local reverse_yaw_error = funcs:yawErrorRad(current_yaw, reverse_target_yaw)
  local freeze_integrator = distance_to_wp <= WP_RADIUS
  local s_out = steering_reverse_pid:compute(reverse_yaw_error, UPDATE_DT, freeze_integrator)

  if math.abs(s_out) < YAW_DEADBAND then
    s_out = 0
  end

  -- applyControlAllocation keeps the same steering sign convention with negative throttle,
  -- so no extra inversion is applied here.
  s_out = funcs:mapMaxMin(s_out, -0.95, 0.95)
  reverse_throttle = funcs:mapMaxMin(reverse_throttle, -0.30, -0.08)
  reverse_status_steps = reverse_status_steps + 1
  if reverse_status_steps >= 10 then
    gcs:send_text(MAV_SEVERITY.INFO,
      string.format("Reverse nav: idx=%d dist=%.1f thr=%.2f steer=%.2f err=%.2f",
        idx, distance_to_wp, reverse_throttle, s_out, reverse_yaw_error))
    reverse_status_steps = 0
  end
  return reverse_throttle, s_out
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
  steering_steady_pid:resetInternalState()
  steering_reverse_pid:resetInternalState()
  reverse_to_next_wp = false
  last_reverse_nav_idx = nil
  reverse_warning_reason = nil
  reverse_status_steps = 0
  reverse_armed_nav_idx = nil
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
    if reverse_leg then
      reverse_armed_nav_idx = idx
      gcs:send_text(MAV_SEVERITY.INFO, string.format("Reverse nav armed: idx=%d", idx))
    end
    if yaw_target_deg == nil then
      resetYawControlState()
      return false
    end
    yaw_target_rad = math.rad(yaw_target_deg)

    -- Reset PID state and start alignment
    steering_steady_pid:resetInternalState()
    steering_reverse_pid:resetInternalState()
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
  resetYawControlState()
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
    steering_reverse_pid:resetInternalState()
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
    steering_steady_pid:resetInternalState()
    steering_reverse_pid:resetInternalState()
    vehicle:set_mode(DRIVING_MODES.AUTO)
    return
  end

  -- Current yaw from AHRS (rad) and error to target
  local err = funcs:yawErrorRad(ahrs:get_yaw(), yaw_target_rad)

  -- Check if we reached the target yaw
  if math.abs(err) <= YAW_THRESH_RAD then
    applyControlAllocation(0, 0)
    steering_steady_pid:resetInternalState()
    steering_reverse_pid:resetInternalState()
    if reverse_to_next_wp then
      vehicle:set_mode(DRIVING_MODES.AUTO)
      return
    end

    -- Set HOLD mode so the vehicle stops before going back to AUTO
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

  -- When starting script in the middle of a mission, infer direction from previous waypoint
  if idx and last_nav_idx == nil and idx > 0 then
    local previous_item = mission:get_item(idx - 1)
    if previous_item and previous_item:command() == 16 then
      local _, reverse_leg, _ = decodeYawAndDirectionFromWaypointZ(previous_item:z())
      reverse_to_next_wp = reverse_leg
      if reverse_leg and reverse_armed_nav_idx ~= idx then
        reverse_armed_nav_idx = idx
        gcs:send_text(MAV_SEVERITY.INFO, string.format("Reverse nav inferred: idx=%d", idx))
      end
    else
      reverse_to_next_wp = false
    end
  end

  -- Controls end of mission
  local mission_state = mission:state()
  if mission_state == MISSION_STATE.FINISHED then
    resetYawControlState()
    applyControlAllocation(0, 0)
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
  else
    steering_reverse_pid:resetInternalState()
    last_reverse_nav_idx = nil
    reverse_warning_reason = nil
    reverse_status_steps = 0
    reverse_armed_nav_idx = nil
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
  -- local p, i, d = param:get('SCR_USER2') / 1000, param:get('SCR_USER3') / 1000, param:get('SCR_USER4') / 1000
  -- steering_steady_pid:setGains(p, i, d)
  -- steering_reverse_pid:setGains(p, i, d)

  -- Getting radio type
  radio_type = param:get('RC3_REVERSED') or 0

  -- Run not armed routine to guarantee trim values
  if not arming:is_armed() then
    resetYawControlState()
    last_nav_idx = nil
    notArmed()
    return update, 2000
  end

  -- Run control output allocation based on the current driving mode
  local vehicle_mode = vehicle:get_mode()
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
  elseif vehicle_mode == DRIVING_MODES.HOLD or vehicle_mode == DRIVING_MODES.GUIDED then
    -- Make the vehicle stop
    steering_reverse_pid:resetInternalState()
    last_reverse_nav_idx = nil
    reverse_warning_reason = nil
    reverse_status_steps = 0
    reverse_armed_nav_idx = nil
    applyControlAllocation(0, 0)
    return update, 200
  end

  steering_reverse_pid:resetInternalState()
  last_reverse_nav_idx = nil
  reverse_warning_reason = nil
  reverse_status_steps = 0
  reverse_armed_nav_idx = nil
end

return update, 3000 -- run immediately before starting to reschedule
-------------------------------------------------------------------------------
