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
local fun = require("functions")
-------------------------------------------------------------------------------
------------------------- GLOBAL SCOPE DEFINITIONS ----------------------------
-------------------------------------------------------------------------------
-- Control variables
local THROTTLE_CONTROL_OUTPUT_CHANNEL = 3
local last_mission_index = -1
-- Throttle smoothing logic
local last_manual_throttle = 0
local throttle_accel_rate_thresh = 0.5
local throttle_accel_rate = 0.5
-- Mission control logic - waypoints XY coordinates to calculate bearing error
local last_wpx, last_wpy = 0, 0
local current_wpx, current_wpy = 0, 0
-- PIDs
-- Params: p_gain, i_gain, d_gain, i_max, i_min, pid_max, pid_min
local ss_pid = PID:new(0.05, 0.01, 0.0, 0.8, -0.8, 0.8, -0.8) -- for simple setpoint control
local lc_pid = PID:new(0.001, 0.03, 0.0, 0.9, -0.9, 0.9, -0.9) -- for line setpoint control
-- Severity for logging in GCS
MAV_SEVERITY = { EMERGENCY = 0, ALERT = 1, CRITICAL = 2, ERROR = 3, WARNING = 4, NOTICE = 5, INFO = 6, DEBUG = 7 }
-- Rover driving modes
DRIVING_MODES = { MANUAL = 0, STEERING = 3, HOLD = 4, AUTO = 10, GUIDED = 15 }
-- Mission states dictionary
MISSION_STATE = { IDLE = 0, RUNNING = 1, FINISHED = 2 }

local function isempty(s)
  return s == nil or s == ''
end
-------------------------------------------------------------------------------

-------------------------------------------------------------------------------
------------------------- LOW LEVEL ACTION FUNCTIONS --------------------------
-------------------------------------------------------------------------------
--[[
Control allocation
This function is responsible for the control allocation of the vehicle.
It receives the throttle and steering values and calculates the PWM values for the motors.
The function also takes into account the trim values for the PWM outputs.
--]]
local function applyControlAllocation(t, s)
  local aloc = 450

  local hip = math.sqrt(t*t + s*s) + 0.0001

  local nTa = aloc * t / hip
  local nSa = aloc * s / hip

  T = math.abs(nTa / (math.abs(nTa) + math.abs(nSa) + 0.0001))
  S = math.abs(nSa / (math.abs(nTa) + math.abs(nSa) + 0.0001))

  local nft = t * T * aloc
  local nfs = s * S * aloc

  local n_aloc_right = math.floor(nft + nfs)
  local n_aloc_left = math.floor(nft - nfs)

 -- Getting the trim values for PWM outputs
  local pwm0_trim_value = tonumber(param:get('SERVO1_TRIM')) or 0
  local pwm1_trim_value = tonumber(param:get('SERVO2_TRIM')) or 0
  local pwm2_trim_value = tonumber(param:get('SERVO3_TRIM')) or 0
  local pwm3_trim_value = tonumber(param:get('SERVO4_TRIM')) or 0

  SRV_Channels:set_output_pwm_chan_timeout(2, pwm2_trim_value + n_aloc_right, 300)
  SRV_Channels:set_output_pwm_chan_timeout(1, pwm1_trim_value + n_aloc_right, 300)
  SRV_Channels:set_output_pwm_chan_timeout(0, pwm0_trim_value - n_aloc_left, 300)
  SRV_Channels:set_output_pwm_chan_timeout(3, pwm3_trim_value - n_aloc_left, 300)
end

--[[
Control the actions while not armed 
--]]
local function notArmed()
  gcs:send_text(MAV_SEVERITY.WARNING, string.format("ROVER - disarmed."))

  local PWM0_TRIM_VALUE = tonumber(param:get('SERVO1_TRIM')) or 0
  local PWM1_TRIM_VALUE = tonumber(param:get('SERVO2_TRIM')) or 0
  local PWM2_TRIM_VALUE = tonumber(param:get('SERVO3_TRIM')) or 0
  local PWM3_TRIM_VALUE = tonumber(param:get('SERVO4_TRIM')) or 0

  SRV_Channels:set_output_pwm_chan_timeout(0, PWM0_TRIM_VALUE, 3000)
  SRV_Channels:set_output_pwm_chan_timeout(1, PWM1_TRIM_VALUE, 3000)
  SRV_Channels:set_output_pwm_chan_timeout(2, PWM2_TRIM_VALUE, 3000)
  SRV_Channels:set_output_pwm_chan_timeout(3, PWM3_TRIM_VALUE, 3000)
end

--[[
Perform vehicle control in Manual mode
--]]
local function manualMode()
  local trim3 = param:get('RC3_TRIM')
  local trim1 = param:get('RC1_TRIM')

  local rc3_pwm = rc:get_pwm(3)
  local rc1_pwm = rc:get_pwm(1)

  local steering = (rc1_pwm - trim1) / 450
  local raw_throttle = (trim3 - rc3_pwm) / 450
  local throttle = raw_throttle
  -- Compares the diff from the last manual throttle to the maximum rate we are accepting
  -- Make the actual command be a rate from the last to the required command if necessary
  if math.abs(last_manual_throttle - raw_throttle) > throttle_accel_rate_thresh then
    throttle = last_manual_throttle + (raw_throttle - last_manual_throttle) * throttle_accel_rate
  end
  last_manual_throttle = throttle

  applyControlAllocation(throttle, steering)
end

-------------------------------------------------------------------------------
--------------------------- MISSION CONTROL SECTION ---------------------------
-------------------------------------------------------------------------------
--[[
Control the outputs using only the bearing to the next waypoint 
--]]
local function simpleSetpointControl()
  local wp_bearing = vehicle:get_wp_bearing_deg()
  local vh_yaw = fun:mapTo360(ahrs:get_yaw()*180.0/3.1415)
  local steering_error = fun:mapError(vh_yaw - wp_bearing)

  local throttle = tonumber(vehicle:get_control_output(THROTTLE_CONTROL_OUTPUT_CHANNEL)) or TRIM3
  local mysteering = ss_pid:compute(0, -steering_error, 0.2)

  return mysteering, throttle
end

--[[
Controls the actions by considering the mission previous and current waypoint, 
and the line between them
--]]
local function getMissionSetpointsData()
  local mission_state = mission:state()
  
  -- Check if mission is over
  if mission_state == MISSION_STATE.FINISHED then
    local steering = 0
    local throttle = 0
    vehicle:set_mode(0)
    last_mission_index = -1

    return steering, throttle
  end

  if last_mission_index == -1 then
    last_mission_index = mission:get_current_nav_index()

    local mylocation = ahrs:get_position()
    last_wpx = mylocation:lat()/1e7
    last_wpy = mylocation:lng()/1e7

    local missionitem = mission:get_item(last_mission_index)
    current_wpx = missionitem:x()/1e7
    current_wpy = missionitem:y()/1e7
  end

  local mission_index = mission:get_current_nav_index()

  if mission_index ~= last_mission_index then
    last_wpx = current_wpx
    last_wpy = current_wpy

    last_mission_index = mission_index;

    local missionitem = mission:get_item(mission_index)
    current_wpx = missionitem:x()/1e7
    current_wpy = missionitem:y()/1e7
  end

  local mylocation = ahrs:get_position()
  local myx = mylocation:lat()/1e7
  local myy = mylocation:lng()/1e7
  local vh_yaw = fun:mapTo360(fun:toDegrees(ahrs:get_yaw()))

  local dist, ang = fun:pointToLineDistance(myx, myy, vh_yaw, last_wpx, last_wpy, current_wpx, current_wpy)

  return dist, ang
end

local function followLineControl()
  local distance, newsteering_error = getMissionSetpointsData()
  local throttle = tonumber(vehicle:get_control_output(THROTTLE_CONTROL_OUTPUT_CHANNEL))

  local mysteering = lc_pid:compute(0, newsteering_error, 0.2)

  return mysteering, throttle
end
-------------------------------------------------------------------------------

-------------------------------------------------------------------------------
-------------------------------- MAIN LOOP ------------------------------------
-------------------------------------------------------------------------------
local function update()
  local vehicle_type = param:get('SCR_USER5')

  if not (vehicle_type == 2) then
    gcs:send_text(MAV_SEVERITY.WARNING, string.format("Not ROVER, exiting LUA script."))
    return
  end

  if not arming:is_armed() then
    notArmed()
    return update, 2000
  end

  -- Controlling in MANUAL MODE
  if vehicle:get_mode() == DRIVING_MODES.MANUAL then
    manualMode()
    return update, 200
  -- Controlling in AUTO MODE
  else
    if vehicle:get_mode() < DRIVING_MODES.AUTO then
      vehicle:set_mode(DRIVING_MODES.AUTO)
    end

    local mission_state = mission:state()
    local lc_steering, lc_throttle = 0, 0 -- for line control method
    local ss_steering, ss_throttle = 0, 0 -- for the simple setpoint method
    ss_steering, ss_throttle = simpleSetpointControl()
    if mission_state == MISSION_STATE.IDLE then
      applyControlAllocation(ss_throttle, ss_steering)
    else
      lc_steering, lc_throttle = followLineControl()
      applyControlAllocation(lc_throttle, (0.4*ss_steering + 0.6*lc_steering))
    end

    return update, 200
  end
end

return update, 3000 -- run immediately before starting to reschedule
-------------------------------------------------------------------------------
