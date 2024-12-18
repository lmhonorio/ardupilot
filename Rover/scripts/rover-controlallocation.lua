

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
-- Control variables
local CONTROL_OUTPUT_THROTTLE = 3
local last_mission_index = -1
local steering = 0
local throttle = 0
-- Throttle smoothing logic
local last_manual_throttle = 0
local throttle_accel_rate_thresh = 0.5
local throttle_accel_rate = 0.5
-- Mission control logic
local last_wpx, last_wpy = 0, 0
local current_wpx, current_wpy = 0, 0
-- PIDs
local steering_pid = PID:new(0.05, 0.01, 0.0, 0.8, -0.8, 0.8, -0.8)
local new_steering_pid = PID:new(0.001, 0.03, 0.0, 0.9, -0.9, 0.9, -0.9)

local function isempty(s)
  return s == nil or s == ''
end

-- ----------------- Control Allocation -----------------
-- This function is responsible for the control allocation of the vehicle.
-- It receives the throttle and steering values and calculates the PWM values for the motors.
-- The function also takes into account the trim values for the PWM outputs.
--@param t number - Throttle value
--@param s number - Steering value
--@return nil
local function newControlAllocation(t, s)
  local aloc = 450

  local hip = math.sqrt(t*t + s*s) + 0.0001

  local nTa = aloc * t / hip
  local nSa = aloc * s / hip

  T = math.abs(nTa / (math.abs(nTa) + math.abs(nSa) + 0.0001))
  S = math.abs(nSa / (math.abs(nTa) + math.abs(nSa) + 0.0001))

  local nft = t * T * aloc
  local nfs = s * S * aloc

  local nalocDir = math.floor(nft + nfs)
  local nalocEsq = math.floor(nft - nfs)

 -- Getting the trim values for PWM outputs
  local pwm0_trim_value = tonumber(param:get('SERVO1_TRIM')) or 0
  local pwm1_trim_value = tonumber(param:get('SERVO2_TRIM')) or 0
  local pwm2_trim_value = tonumber(param:get('SERVO3_TRIM')) or 0
  local pwm3_trim_value = tonumber(param:get('SERVO4_TRIM')) or 0

  SRV_Channels:set_output_pwm_chan_timeout(2, pwm2_trim_value + nalocDir, 300)
  SRV_Channels:set_output_pwm_chan_timeout(1, pwm1_trim_value + nalocDir, 300)
  SRV_Channels:set_output_pwm_chan_timeout(0, pwm0_trim_value - nalocEsq, 300)
  SRV_Channels:set_output_pwm_chan_timeout(3, pwm3_trim_value - nalocEsq, 300)
end

--[[
Control the actions while not armed 
--]]
local function notArmed()
  gcs:send_text(4, string.format("ROVER - desarmado "))

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

  steering = (rc1_pwm - trim1) / 450
  local raw_throttle = (trim3 - rc3_pwm) / 450
  throttle = raw_throttle
  -- Compares the diff from the last manual throttle to the maximum rate we are accepting
  -- Make the actual command be a rate from the last to the required command if necessary
  if math.abs(last_manual_throttle - raw_throttle) > throttle_accel_rate_thresh then
    throttle = last_manual_throttle + (raw_throttle - last_manual_throttle) * throttle_accel_rate
  end
  last_manual_throttle = throttle

  newControlAllocation(throttle, steering)
end

-------------------------------------------------------------------------------
--------------------------- MISSION CONTROL SECTION ---------------------------
-------------------------------------------------------------------------------
--[[
Control the outputs using only the bearing to the next waypoint 
--]]
local function updateSimpleSetpoints()
  local wp_bearing = vehicle:get_wp_bearing_deg()
  local vh_yaw = fun:map_to_360(ahrs:get_yaw()*180.0/3.1415)
  local steering_error = fun:map_error(vh_yaw - wp_bearing)

  throttle = tonumber(vehicle:get_control_output(CONTROL_OUTPUT_THROTTLE)) or TRIM3
  local mysteering = steering_pid:compute(0, -steering_error, 0.2)

  return mysteering, throttle
end

--[[
Controls the actions by considering the mission previous and current waypoint, 
and the line between them
--]]
local function updateMissionSetpoints()
  local mission_state = mission:state()
  
  -- Check if mission is over
  if mission_state == 2 then
    steering = 0
    throttle = 0
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
  local vh_yaw = fun:map_to_360(fun:to_degrees(ahrs:get_yaw()))

  local dist, ang = fun:point_to_line_distance(myx, myy, vh_yaw, last_wpx, last_wpy, current_wpx, current_wpy)

  return dist, ang
end

local function updateFollowLine()
  local distance, newsteering_error = updateMissionSetpoints()
  throttle = tonumber(vehicle:get_control_output(CONTROL_OUTPUT_THROTTLE))

  local mysteering = new_steering_pid:compute(0, newsteering_error, 0.2)

  return mysteering, throttle
end
-------------------------------------------------------------------------------

-------------------------------------------------------------------------------
-------------------------------- MAIN LOOP ------------------------------------
-------------------------------------------------------------------------------
local function update()
  local vehicle_type = param:get('SCR_USER5')

  if not (vehicle_type==2) then
    gcs:send_text(4, string.format("Not ROVER, exiting LUA script."))
    return
  end

  if not arming:is_armed() then
    notArmed()
    return update, 2000
  end

  if vehicle:get_mode() == 0 then
    manualMode()
    return update, 200
  else
    if vehicle:get_mode() < 10 then
      vehicle:set_mode(10)
    end

    local mission_state = mission:state()
    local newsteering, newthrottle

    if mission_state == 0 then
      steering, throttle = updateSimpleSetpoints()
      newsteering, newthrottle = steering, throttle
    else
      steering, throttle = updateSimpleSetpoints()
      newsteering, newthrottle = updateFollowLine()
    end

    newControlAllocation(newthrottle, (0.4*steering + 0.6*newsteering))

    return update, 200
  end
end

return update, 3000 -- run immediately before starting to reschedule
-------------------------------------------------------------------------------
