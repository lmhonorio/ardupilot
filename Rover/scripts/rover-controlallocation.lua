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
-- Vehicle type control
local VEHICLE_TYPE = param:get('SCR_USER5')
-- TRIM values
local PWM_TRIM_VALUE = tonumber(param:get('SERVO1_TRIM')) or 0
local RC1_TRIM_VALUE = param:get('RC1_TRIM')
local RC3_TRIM_VALUE = param:get('RC3_TRIM')
-- DEAD ZONE thresh
local DEAD_ZONE_THRESH = 40
-- Signal smoothing logic
local last_manual_throttle = 0
local throttle_accel_rate_thresh = 0.5
local throttle_accel_rate = 0.5
local last_manual_steering = 0
local steering_accel_rate_thresh = 0.6
local steering_accel_rate = 0.6
-- Mission control logic - waypoints XY coordinates to calculate bearing error and setpoints
local current_wp_x, current_wp_y = 0, 0
local next_wp_x, next_wp_y = 0, 0
-- Distance to consider a waypoint reached [m]
local thresh_dist_wp_reached = 2
-- Dead zone for steering when very close to waypoint [m]
local thresh_dist_steering_deadzone = 0.5
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
This function is responsible for the PWM signal allocation of the vehicle.
It receives the throttle (t) and steering (s) values and calculates the PWM values for the motors.
The function also takes into account the trim values for the PWM outputs.
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

  -- First time the mission is running, start the last mission index with the first 
  -- and second waypoints to mark the line
  if current_mission_index == -1 then
    current_mission_index = mission:get_current_nav_index()
    local next_mission_index = current_mission_index + 1

    -- Verify if there's a next waypoint available
    local next_waypoint = mission:get_item(next_mission_index)
    if next_waypoint then
      -- last_wp = current waypoint (where we are going from)
      local current_waypoint = mission:get_item(current_mission_index)
      current_wp_x = current_waypoint:x() / 1e7
      current_wp_y = current_waypoint:y() / 1e7

      -- current_wp = next waypoint (where we are going to)
      next_wp_x = next_waypoint:x() / 1e7
      next_wp_y = next_waypoint:y() / 1e7
    else
      -- No next waypoint, we're at the final waypoint already
      return 0
    end
  end

  -- If we switched waypoints, refresh the last and current waypoints to form the line
  local mission_index = mission:get_current_nav_index()
  if mission_index ~= current_mission_index then
    -- Update to new waypoint pair
    local next_mission_index = mission_index + 1
    local next_waypoint = mission:get_item(next_mission_index)
    
    if next_waypoint then
      -- last_wp = current waypoint (where we just arrived or are arriving)
      current_mission_index = mission_index
      local current_waypoint = mission:get_item(current_mission_index)
      current_wp_x = current_waypoint:x() / 1e7
      current_wp_y = current_waypoint:y() / 1e7
      
      -- current_wp = next waypoint (where we are going next)
      next_wp_x = next_waypoint:x() / 1e7
      next_wp_y = next_waypoint:y() / 1e7
    else
      -- No next waypoint, we're at the final waypoint already
      return 0
    end
  end

  -- Get vehicle location info
  local vh_location = ahrs:get_position()
  local vh_x = vh_location:lng() / 1e7
  local vh_y = vh_location:lat() / 1e7
  local vh_yaw = funcs:mapTo360(funcs:toDegrees(ahrs:get_yaw()))

  -- Vehicle velocity info
  local vh_velocity = ahrs:groundspeed_vector()
  local vh_velocity_norm = math.sqrt(vh_velocity:x() ^ 2 + vh_velocity:y() ^ 2)

  -- In case of any nil value from the internal state, do not proceed yet
  if vh_x == nil or current_wp_x == nil or next_wp_x == nil then
    return 0
  end

  -- Get the projected point with some lookahead so we keep pursuing the target waypoint direction
  local line_point_x, line_point_y = funcs:lineProjectionPoint(vh_x, vh_y, current_wp_x, current_wp_y, next_wp_x,
    next_wp_y)
  -- The bearing angle between the last and current waypoints
  local wp_line_bearing = funcs:calculateBearingBetweenPoints(current_wp_x, current_wp_y, next_wp_x, next_wp_y)
  gcs:send_text(MAV_SEVERITY.WARNING, string.format("WP line bearing: %f", wp_line_bearing))
  local heading_error = funcs:mapErrorToRange(wp_line_bearing - vh_yaw)
  gcs:send_text(MAV_SEVERITY.WARNING, string.format("HEA: %.2f deg", heading_error))
  -- Calculate the cross track error from the vehicle to the line between waypoints
  local cross_track_error_gain = param:get('SCR_USER6')
  local cross_track_error = funcs:crossTrackError(vh_velocity_norm, cross_track_error_gain, line_point_x, line_point_y, vh_x, vh_y)
  local cross_track_error_sign = funcs:lineSideSignal(current_wp_x, current_wp_y, next_wp_x, next_wp_y, vh_x, vh_y)
  gcs:send_text(MAV_SEVERITY.WARNING, string.format("CTE sign: %.2f m", cross_track_error_sign))
  -- Return the steering error as the sum of both errors
  local steering_error = funcs:mapErrorToRange(heading_error + cross_track_error_sign * cross_track_error)
  gcs:send_text(MAV_SEVERITY.WARNING, string.format("CTE: %.2f m", cross_track_error_sign * cross_track_error))
  gcs:send_text(MAV_SEVERITY.WARNING, string.format("STE: %.2f deg", steering_error))

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
    current_wp_x, current_wp_y = 0, 0
    next_wp_x, next_wp_y = 0, 0
    -- Resetting PIDs
    -- We must reset integrator and last error if not using the pid in AUTO mode anymore
    ss_pid:resetInternalState()
    lc_pid:resetInternalState()
    applyPWMManualMode()
    return update, 200
  else
    --[[
    Controlling in AUTO MODE
    --]]
    if vehicle:get_mode() < DRIVING_MODES.AUTO then
      vehicle:set_mode(DRIVING_MODES.AUTO)
    end

    -- Acquiring throttle from internal control output
    local throttle = tonumber(vehicle:get_control_output(THROTTLE_CONTROL_OUTPUT_CHANNEL))
    throttle = funcs:mapMaxMin(throttle, 0.1, 1.0)

    -- Getting steering from two methods:
    -- We should pursue both the final waypoint and the current location projected in the line
    -- That guarantees a fast response, but also a smooth transition between waypoints
    -- The weighted sum of these values should be the final steering command
    local ss_rate = param:get('SCR_USER1')
    local ss_steering = simpleSetpointControl()
    local lc_rate = 1.0 - ss_rate
    local lc_steering = followLineControl()

    -- Fetch the current and target position of the vehicle
    local vehicle_position = ahrs:get_position()
    local target_position = vehicle:get_target_location()
    -- Verify which error to consider for steering
    local steering_error = 0 -- Should be 0 if close enough to the waypoint indeed
    if vehicle_position and target_position then
      -- Calculate the distance to the target (in meters) and choose which controller to use
      local dist = vehicle_position:get_distance(target_position)
      gcs:send_text(MAV_SEVERITY.WARNING, string.format("Dist to WP: %.1f m", dist))
      if dist > thresh_dist_wp_reached then
        steering_error = ss_rate * ss_steering + lc_rate * lc_steering
      end
    else
      steering_error = ss_rate * ss_steering + lc_rate * lc_steering
    end

    -- Apply the control allocation finally
    applyControlAllocation(throttle, steering_error)

    return update, 200
  end
end

return update, 3000 -- run immediately before starting to reschedule
-------------------------------------------------------------------------------
