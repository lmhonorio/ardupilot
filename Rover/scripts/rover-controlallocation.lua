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
local last_mission_index = -1
-- Vehicle type control
local VEHICLE_TYPE = param:get('SCR_USER5')
-- TRIM values
local PWM0_TRIM_VALUE = tonumber(param:get('SERVO1_TRIM')) or 0
local PWM1_TRIM_VALUE = tonumber(param:get('SERVO2_TRIM')) or 0
local PWM2_TRIM_VALUE = tonumber(param:get('SERVO3_TRIM')) or 0
local PWM3_TRIM_VALUE = tonumber(param:get('SERVO4_TRIM')) or 0
local RC1_TRIM_VALUE = param:get('RC1_TRIM')
local RC3_TRIM_VALUE = param:get('RC3_TRIM')
-- DEAD ZONE thresh
local DEAD_ZONE_THRESH = 40
-- Signal smoothing logic
local last_manual_throttle = 0
local throttle_accel_rate_thresh = 0.5
local throttle_accel_rate = 0.5
local last_manual_steering = 0
local steering_accel_rate_thresh = 0.5
local steering_accel_rate = 0.5
-- Mission control logic - waypoints XY coordinates to calculate bearing error and setpoints
local last_wp_x, last_wp_y = 0, 0
local current_wp_x, current_wp_y = 0, 0
-- PIDs
-- Params: p_gain, i_gain, d_gain, i_max, i_min, pid_max, pid_min
local ss_pid = PID:new(0.05, 0.01, 0.0, 80, -80, 0.99, -0.99)  -- for simple setpoint control
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
  -- The throttle and steering absolute sum, to inspect the total signal we want to insert
  local ts_sum = math.abs(t) + math.abs(s) + 0.00001
  -- We compare the value of each input to the total sum
  local t_share, s_share = math.abs(t) / ts_sum, math.abs(s) / ts_sum
  -- We create the assigned amount of steering and throttle by multiplying again the shares by the inputs
  -- We do that to take the input signal into account, and also to
  -- reduce the output a bit so we dont send the full signal to the motors all the time,
  -- as if it would happen if only using the proportion of the sum
  t_share, s_share = t * t_share, s * s_share
  -- The right and left motors added PWM allocation
  local pwm_aloc_r = (t_share + s_share) * PWM_RANGE
  local pwm_aloc_l = (t_share - s_share) * PWM_RANGE
  -- Limiting the output values to the PWM ranges
  local pwm_0 = funcs:mapMaxMin(PWM0_TRIM_VALUE - pwm_aloc_l, MIN_CHANNEL_OUTPUT, MAX_CHANNEL_OUTPUT)
  local pwm_1 = funcs:mapMaxMin(PWM1_TRIM_VALUE + pwm_aloc_r, MIN_CHANNEL_OUTPUT, MAX_CHANNEL_OUTPUT)
  local pwm_2 = funcs:mapMaxMin(PWM2_TRIM_VALUE + pwm_aloc_r, MIN_CHANNEL_OUTPUT, MAX_CHANNEL_OUTPUT)
  local pwm_3 = funcs:mapMaxMin(PWM3_TRIM_VALUE - pwm_aloc_l, MIN_CHANNEL_OUTPUT, MAX_CHANNEL_OUTPUT)
  -- Check if the values are within the dead zone, and if so, set the outputs to the trim values
  -- pwm_0 = funcs:applyDeadZone(pwm_0, PWM0_TRIM_VALUE, DEAD_ZONE_THRESH)
  -- pwm_1 = funcs:applyDeadZone(pwm_1, PWM1_TRIM_VALUE, DEAD_ZONE_THRESH)
  -- pwm_2 = funcs:applyDeadZone(pwm_2, PWM2_TRIM_VALUE, DEAD_ZONE_THRESH)
  -- pwm_3 = funcs:applyDeadZone(pwm_3, PWM3_TRIM_VALUE, DEAD_ZONE_THRESH)
  -- Setting the PWM outputs based on the control allocation directions
  SRV_Channels:set_output_pwm_chan_timeout(0, pwm_0, 300)
  SRV_Channels:set_output_pwm_chan_timeout(1, pwm_1, 300)
  SRV_Channels:set_output_pwm_chan_timeout(2, pwm_2, 300)
  SRV_Channels:set_output_pwm_chan_timeout(3, pwm_3, 300)
end

--[[
Control the actions while not armed
--]]
local function notArmed()
  SRV_Channels:set_output_pwm_chan_timeout(0, PWM0_TRIM_VALUE, 3000)
  SRV_Channels:set_output_pwm_chan_timeout(1, PWM1_TRIM_VALUE, 3000)
  SRV_Channels:set_output_pwm_chan_timeout(2, PWM2_TRIM_VALUE, 3000)
  SRV_Channels:set_output_pwm_chan_timeout(3, PWM3_TRIM_VALUE, 3000)
end

--[[
Perform vehicle control in Manual mode
--]]
local function manualMode()
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
    last_mission_index = -1

    return steering, throttle
  end
  -- First time the mission is running, start the last mission index with the first waypoint
  -- Use current location as the reference last waypoint
  if last_mission_index == -1 then
    last_mission_index = mission:get_current_nav_index()

    local vh_location = ahrs:get_position()
    last_wp_x = vh_location:lat() / 1e7
    last_wp_y = vh_location:lng() / 1e7

    local current_waypoint = mission:get_item(last_mission_index)
    current_wp_x = current_waypoint:x() / 1e7
    current_wp_y = current_waypoint:y() / 1e7
  end

  -- If we switched waypoints, refresh the last and current waypoints to form the line
  local mission_index = mission:get_current_nav_index()
  if mission_index ~= last_mission_index then
    last_wp_x = current_wp_x
    last_wp_y = current_wp_y

    last_mission_index = mission_index;

    local current_waypoint = mission:get_item(mission_index)
    current_wp_x = current_waypoint:x() / 1e7
    current_wp_y = current_waypoint:y() / 1e7
  end

  -- Get vehicle location info
  local vh_location = ahrs:get_position()
  local vh_x = vh_location:lat() / 1e7
  local vh_y = vh_location:lng() / 1e7
  local vh_yaw = funcs:mapTo360(funcs:toDegrees(ahrs:get_yaw()))

  -- In case of any nil value from the internal state, do not proceed yet
  if vh_x == nil or last_wp_x == nil or current_wp_x == nil then
    return 0
  end

  -- Get the projected point with some lookahead so we keep pursuing the target waypoint direction
  local line_point_x, line_point_y = funcs:lineProjectionPoint(vh_x, vh_y, last_wp_x, last_wp_y, current_wp_x,
    current_wp_y)
  -- The bearing angle to the line projection
  local vh_line_bearing = funcs:calculateBearingBetweenPoints(vh_x, vh_y, line_point_x, line_point_y)
  -- Return the steering error from the vehicle yaw to the desired bearing
  local steering_error = funcs:mapErrorToRange(vh_line_bearing - vh_yaw)
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
  ss_pid:setGains(p, i, d)
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
    -- We must reset integrator and last error if not using the pid in AUTO mode anymore
    ss_pid:resetInternalState()
    lc_pid:resetInternalState()
    manualMode()
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
    -- We should pursue both the final waypoint and the current location projected in the line with a small lookahead
    -- That guarantees a fast response, but also a smooth transition between waypoints
    -- The weighted sum of these values should be the final steering command
    local ss_rate = param:get('SCR_USER1')
    local ss_steering = simpleSetpointControl()
    local lc_rate = 1.0 - ss_rate
    local lc_steering = followLineControl()

    local steering_error = ss_rate * ss_steering + lc_rate * lc_steering
    applyControlAllocation(throttle, steering_error)

    return update, 200
  end
end

return update, 3000 -- run immediately before starting to reschedule
-------------------------------------------------------------------------------
