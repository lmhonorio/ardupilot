

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
package.path = package.path .. ';./scripts/modules/?.lua'
local PID = require("pid")
local fun = require("functions")


local CONTROL_OUTPUT_THROTTLE = 3
local CONTROL_OUTPUT_YAW = 4

local THROTTLE_LEFT_id = 73
local THROTTLE_RIGHT_id = 74
local THROTTLE_LEFT_COMMAND = SRV_Channels:find_channel(73)
local THROTTLE_RIGHT_COMMAND = SRV_Channels:find_channel(74)

local last_mission_index =  -1


local throttle_output = 0
local steering_output = 0

local steering = 0
local throttle = 0

local trim3 
local trim1 



local function isempty(s)
  return s == nil or s == ''
end


local function CtrlAlocacaonovo(t, s)

  
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


  local PWM0_TRIM_VALUE = param:get('SERVO1_TRIM')
  local PWM2_TRIM_VALUE = param:get('SERVO3_TRIM')


  SRV_Channels:set_output_pwm_chan_timeout(0, nalocDir+PWM2_TRIM_VALUE, 300)
  SRV_Channels:set_output_pwm_chan_timeout(2, nalocEsq+PWM0_TRIM_VALUE, 300)


  
end


local last_wpx, last_wpy = 0,0
local current_wpx, current_wpy = 0,0


function not_armed()
      --vehicle:set_mode(15)
      gcs:send_text(4, string.format("ROVER - desarmado "))

      local PWM0_TRIM_VALUE = param:get('SERVO1_TRIM')
      local PWM1_TRIM_VALUE = param:get('SERVO2_TRIM')
      local PWM2_TRIM_VALUE = param:get('SERVO3_TRIM')
      local PWM3_TRIM_VALUE = param:get('SERVO4_TRIM')
  
      -- SRV_Channels:set_output_pwm_chan_timeout(THROTTLE_LEFT_COMMAND, 1500,300) 
      -- SRV_Channels:set_output_pwm_chan_timeout(THROTTLE_RIGHT_COMMAND,1500,300)
  
   
      SRV_Channels:set_output_pwm_chan_timeout(0,PWM0_TRIM_VALUE,3000) 
      SRV_Channels:set_output_pwm_chan_timeout(1,PWM1_TRIM_VALUE,3000)
      SRV_Channels:set_output_pwm_chan_timeout(2,PWM2_TRIM_VALUE,3000) 
      SRV_Channels:set_output_pwm_chan_timeout(3,PWM3_TRIM_VALUE,3000)
end

function Manual_mode()

  local rc3_pwm = 0
  local rc1_pwm = 0

  trim3 = param:get('RC3_TRIM')
  trim1 = param:get('RC1_TRIM')

  rc3_pwm = rc:get_pwm(3)
  rc1_pwm = rc:get_pwm(1)

  throttle = (trim3 - rc3_pwm) / 450
  steering = (rc1_pwm - trim1) / 450

  CtrlAlocacaonovo(throttle,steering)
  
end

local steering_pid = PID:new(0.05, 0.01, 0.0, 0.8, -0.8, 0.8, -0.8)  -- Configure os ganhos como necessários


function Update_simple_setpoints()

  local wp_bearing = vehicle:get_wp_bearing_deg()
  local vh_yaw = fun:map_to_360(ahrs:get_yaw()*180.0/3.1415)
  local steering_error = fun:map_error(vh_yaw - wp_bearing)

  --steering = vehicle:get_control_output(CONTROL_OUTPUT_YAW)
  throttle = tonumber(vehicle:get_control_output(CONTROL_OUTPUT_THROTTLE))

  local mysteering = steering_pid:compute(0,-steering_error)

  return mysteering, throttle

end


local newsteering_pid = PID:new(0.001, 0.03, 0.0, 0.9, -0.9, 0.9, -0.9)  



function Update_mission_setpoints()

  local mission_state = mission:state()
  --[[
    Verifica se terminou a missao
  --]]
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

    gcs:send_text(4, "LUA: New Mission Item") -- we spotted a change

    last_wpx = current_wpx
    last_wpy = current_wpy

    last_mission_index = mission_index;

    local missionitem = mission:get_item(mission_index)
    current_wpx = missionitem:x()/1e7
    current_wpy= missionitem:y()/1e7

  end

  local mylocation = ahrs:get_position()
  local myx = mylocation:lat()/1e7
  local myy = mylocation:lng()/1e7

  local vh_yaw = fun:map_to_360(fun:To_degrees(ahrs:get_yaw()))


  local dist, ang = fun:Point_to_line_distance(myx, myy, vh_yaw, last_wpx, last_wpy, current_wpx, current_wpy)

  return dist, ang
  
end



function update_follow_line()

  local distance,newsteering_error = Update_mission_setpoints()
  --steering = vehicle:get_control_output(CONTROL_OUTPUT_YAW)
  throttle = tonumber(vehicle:get_control_output(CONTROL_OUTPUT_THROTTLE))

  local mysteering = newsteering_pid:compute(0,newsteering_error)

  return mysteering, throttle

end






function update() -- this is the loop which periodically runs

  local tipoveiculo = param:get('SCR_USER5')

  if not (tipoveiculo==2) then
    gcs:send_text(4, string.format("nao e ROVER saindo do lua"))
    return
  end

  if not arming:is_armed() then
    
    not_armed()

    return update, 2000
  end

    -- steering = vehicle:get_control_output(CONTROL_OUTPUT_YAW) 
    -- throttle = vehicle:get_control_output(CONTROL_OUTPUT_THROTTLE)

  if vehicle:get_mode()== 0 then

    Manual_mode()
    return update, 200

  else

    if vehicle:get_mode()< 10 then
      vehicle:set_mode(10)
    end
    

    local mission_state = mission:state()
    local newsteering, newthrottle

    if mission_state == 0 then
      steering, throttle = Update_simple_setpoints()
      local testtarget = vehicle:get_wp_distance_m()
      local testbearing = vehicle:get_wp_bearing_deg()
      newsteering, newthrottle =steering, throttle

    else
      steering, throttle = Update_simple_setpoints()
      newsteering, newthrottle = update_follow_line()
      
      
      --gcs:send_text(4, string.format("distancia ao SR %f - ang %f",ajdthrotte,ajdsteering) )

    end
    

    --steering, throttle = update_mission_setpoints()

    CtrlAlocacaonovo(newthrottle, (0.4*steering+ 0.6*newsteering))
      

    --CtrlAlocacaonovo(throttle,steering)


      return update, 200

  end

end

return update, 3000 -- run immediately before starting to reschedule
