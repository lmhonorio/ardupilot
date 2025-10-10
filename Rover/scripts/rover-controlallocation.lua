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
local MAX_CHANNEL_OUTPUT = 1950
local MIN_CHANNEL_OUTPUT = 1050
local PWM_RANGE = 450
local current_mission_index = -1 -- The index we have just reached in the mission
local current_mission_index_controls_reset = -1 -- The index we have just reached in the mission to reset controls
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
-- Mission control logic - waypoints XY coordinates to calculate bearing error and setpoints
local current_wp_lat, current_wp_lon = 0, 0
local last_wp_lat, last_wp_lon = 0, 0
-- PIDs
-- Params: p_gain, i_gain, d_gain, i_max, i_min, pid_max, pid_min
local ss_pid = PID:new(0.050, 0, 0.010, 90, -90, 0.99, -0.99)  -- for simple setpoint control
local lc_pid = PID:new(0.001, 0.03, 0.0, 90, -90, 0.99, -0.99) -- for line setpoint control
-- Severity for logging in GCS
MAV_SEVERITY = { EMERGENCY = 0, ALERT = 1, CRITICAL = 2, ERROR = 3, WARNING = 4, NOTICE = 5, INFO = 6, DEBUG = 7 }
-- Rover driving modes
DRIVING_MODES = { MANUAL = 0, STEERING = 3, HOLD = 4, AUTO = 10, GUIDED = 15 }
-- Mission states dictionary
MISSION_STATE = { IDLE = 0, RUNNING = 1, FINISHED = 2 }
-------------------------------------------------------------------------------

-------------------------------------------------------------------------------
------------------------- LOW LEVEL ACTION FUNCTIONS --------------------------
-------------------------------------------------------------------------------
--[[
Control allocation
This function is responsible for the PWM signal allocation of the vehicle in manual mode
It receives the throttle (t) and steering (s) values and calculates the PWM values for the motors.
The function also takes into account the trim values for the PWM outputs.
-- @param t number - Throttle command from 0 (or more) to 1.0
-- @param s number - Steering command from -1.0 to 1.0
--]]
local function ApplyControlAllocationManually(t, s)
  -- We assign the PWM values to the motors, which are opposite in sign for each diagonal pair
  -- MOTOR SCHEMATIC IN ROVER FRAME
  -- 1 - 0     ^
  --   |       | Rover forward direction
  -- 2 - 3
  local pwm_aloc_l, pwm_aloc_r = funcs:allocateRightAndLeftPwmShareManually(t, s, PWM_RANGE)
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

--[[
Control allocation in AUTO mode, where we only need to get the steering value, 
and with a constant diagonal of forces we can calculate the available part for throttle
-- @param steering number - Steering command from -1.0 to 1.0
--]]
local function applyControlAllocationAutoMode(steering)
  -- We assign the PWM values to the motors, which are opposite in sign for each diagonal pair
  -- MOTOR SCHEMATIC IN ROVER FRAME
  -- 1 - 0     ^
  --   |       | Rover forward direction
  -- 2 - 3
  local forces_diagonal = 1.0
  -- Pythagorean theorem to get the available throttle
  local throttle = math.sqrt(forces_diagonal - steering * steering)
  throttle = funcs:mapMaxMin(throttle, 0.1, 1.0) -- make sure we dont stall in the same spot
  -- Getting each share in PWM values, throttle and steering will never go above 1.0
  local pwm_aloc_l = math.floor((throttle + steering) * PWM_RANGE)
  local pwm_aloc_r = math.floor((throttle - steering) * PWM_RANGE)
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

  ApplyControlAllocationManually(throttle, steering)
end

--[[
Calculates a line and a bearing from the vehicle to the line projection
--]]
local function getLineBearingFromWaypoints()
  local mission_state = mission:state()
  -- Check if mission is over
  if mission_state == MISSION_STATE.FINISHED then
    local steering = 0
    local throttle = 0
    vehicle:set_mode(0)
    current_mission_index = -1

    return steering, throttle
  end

  -- First time when we start the mission, use current vehicle position as waypoint
  if current_mission_index == -1 then
    local vh_location = ahrs:get_position()
    if vh_location then
      last_wp_lat = vh_location:lat() / 1e7
      last_wp_lon = vh_location:lng() / 1e7
      
      current_mission_index = mission:get_current_nav_index()
      local current_waypoint = mission:get_item(current_mission_index)
      current_wp_lat = current_waypoint:x() / 1e7
      current_wp_lon = current_waypoint:y() / 1e7
    end
  end

  -- If we switched waypoints, refresh the last and current waypoints to form the line
  local mission_index = mission:get_current_nav_index()
  if mission_index ~= current_mission_index then
    -- Update the last and current waypoints
    last_wp_lat = current_wp_lat
    last_wp_lon = current_wp_lon
    current_mission_index = mission_index
    local current_waypoint = mission:get_item(mission_index)
    if current_waypoint then
      current_wp_lat = current_waypoint:x() / 1e7
      current_wp_lon = current_waypoint:y() / 1e7
    end
  end

  -- Get vehicle location info
  local vh_location = ahrs:get_position()
  local vh_lon = vh_location:lng() / 1e7
  local vh_lat = vh_location:lat() / 1e7
  local vh_yaw = funcs:mapTo360(funcs:toDegrees(ahrs:get_yaw()))

  -- Vehicle velocity info
  local vh_velocity = ahrs:groundspeed_vector()
  local vh_velocity_norm = math.sqrt(vh_velocity:x() ^ 2 + vh_velocity:y() ^ 2)

  -- In case of any nil value from the internal state, do not proceed yet
  if vh_lon == nil or current_wp_lat == nil or last_wp_lat == nil then
    return 0
  end

  local line_point_lon, line_point_lat = funcs:lineProjectionPoint(vh_lon, vh_lat, last_wp_lon, last_wp_lat,
    current_wp_lon, current_wp_lat)
  -- The bearing angle between the last and current waypoints
  local wp_line_bearing = funcs:calculateBearingBetweenPoints(last_wp_lat, last_wp_lon, current_wp_lat,
    current_wp_lon)
  if wp_line_bearing == 90.0 then
    return 0 -- Case of very close waypoints
  end
  local heading_error = funcs:mapErrorToRange(wp_line_bearing - vh_yaw)
  -- Calculate the cross track error from the vehicle to the line between waypoints
  local cross_track_error_gain = param:get('SCR_USER6')
  local cross_track_error = funcs:crossTrackError(vh_velocity_norm, cross_track_error_gain, line_point_lat, line_point_lon, vh_lat, vh_lon)
  local cross_track_error_sign = funcs:lineSideSignal(last_wp_lon, last_wp_lat, current_wp_lon, current_wp_lat, vh_lon, vh_lat)
  -- Return the steering error as the sum of both errors
  local steering_error = funcs:mapErrorToRange(heading_error + cross_track_error_sign * cross_track_error)

  return steering_error
end
-------------------------------------------------------------------------------

-------------------------------------------------------------------------------
--------------------------- MISSION CONTROL SECTION ---------------------------
-------------------------------------------------------------------------------
--[[
Control the outputs using only the bearing to the next waypoint
--]]
local function simpleSetpointControl()
  -- We have the yaw returned in degrees, 0 degrees to the North, 90 to the East
  -- The bearing to waypoint has its origin on the vehicle, also following North-East frame
  local wp_bearing = vehicle:get_wp_bearing_deg()
  local vh_yaw = funcs:mapTo360(ahrs:get_yaw() * 180.0 / 3.1415)
  local steering_error = funcs:mapErrorToRange(wp_bearing - vh_yaw)

  -- Compute the steering command signal with the PID simple setpoint controller
  local steering = ss_pid:compute(steering_error, 0.2)
  return steering
end

--[[
Controls the actions by considering the mission previous and current waypoint,
and the line between them
--]]
local function followLineControl()
  local steering_error = getLineBearingFromWaypoints()
  local steering = lc_pid:compute(steering_error, 0.2)
  return steering
end
-------------------------------------------------------------------------------

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
  -- ss_pid:setGains(p, i, d)
  lc_pid:setGains(p, i, d)

  -- Run not armed routine to guarantee trim values
  if not arming:is_armed() then
    notArmed()
    return update, 2000
  end

  if vehicle:get_mode() == DRIVING_MODES.MANUAL then
    --[[
    Controlling in MANUAL MODE
    --]]
    -- Reseting control variables
    current_mission_index = -1
    current_wp_lat, current_wp_lon = 0, 0
    last_wp_lat, last_wp_lon = 0, 0
    -- Resetting PIDs
    ss_pid:resetInternalState()
    lc_pid:resetInternalState()
    applyPWMManualMode()
    return update, 200
  elseif vehicle:get_mode() == DRIVING_MODES.HOLD then
    --[[
    Controlling in HOLD MODE
    --]]
    -- Resetting PIDs
    ss_pid:resetInternalState()
    lc_pid:resetInternalState()
    -- Make the vehicle stop
    ApplyControlAllocationManually(0, 0)
    return update, 200
  elseif vehicle:get_mode() == DRIVING_MODES.AUTO then
    --[[
    Controlling in AUTO MODE
    --]]

    -- Check if the vehicle waypoint has changed, and if so,
    -- reset the PIDs internal state to avoid spikes and send a 0 signal to the motors
    local mission_index = mission:get_current_nav_index()
    if mission_index ~= current_mission_index_controls_reset and mission_index > 0 then
      ss_pid:resetInternalState()
      lc_pid:resetInternalState()
      ApplyControlAllocationManually(0, 0)
      current_mission_index_controls_reset = mission_index
      return update, 200
    end

    -- Getting steering from two methods:
    -- We should pursue both the final waypoint and the current location projected in the line
    -- That guarantees a fast response, but also a smooth transition between waypoints
    -- The weighted sum of these values should be the final steering command
    local ss_rate = param:get('SCR_USER1')
    local ss_steering = simpleSetpointControl()
    local lc_rate = 1.0 - ss_rate
    local lc_steering = followLineControl()
    local steering_error = ss_rate * ss_steering + lc_rate * lc_steering

    -- Apply the control allocation finally
    applyControlAllocationAutoMode(steering_error)

    return update, 200
  end
end

return update, 3000 -- run immediately before starting to reschedule
-------------------------------------------------------------------------------
