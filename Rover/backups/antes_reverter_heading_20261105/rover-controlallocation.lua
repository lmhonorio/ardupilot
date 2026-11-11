-- Alocacao diferencial: aplica aos motores os comandos do controlador ArduRover.
-- O FAST-LIO permanece em lidar_odometry.lua e nao interfere na atuacao.
package.path = package.path .. ';./scripts/modules/?.lua'
local funcs = require('functions')
local final_heading = require('heading_final')
local THROTTLE_CONTROL_OUTPUT_CHANNEL, CONTROL_OUTPUT_YAW = 3, 4
local UPDATE_PERIOD_MS, OUTPUT_TIMEOUT_MS = 20, 100
local MAV_SEVERITY = {ERROR=3, WARNING=4}
local motor_outputs, outputs_overridden, outputs_loaded = {}, false, false
local vehicle_type_warning = false
local allocation_param, allocation_bound = Parameter(), false
local final_state = 'idle'
local final_target, final_yaw = nil, nil
local final_ticks, aligned_ticks = 0, 0
local final_wait_warned = false

local function resetFinalAlignment()
  final_state='idle'; final_target=nil; final_yaw=nil; final_ticks=0; aligned_ticks=0; final_wait_warned=false
end

local function nearbyClear()
  local prox_configured=(param:get('PRX1_TYPE') or 0) ~= 0 or (param:get('OA_TYPE') or 0) ~= 0
  if not prox_configured then return true end
  if proximity == nil or proximity:get_status() ~= 2 then return false end
  local _,distance=proximity:get_closest_object()
  return distance == nil or distance >= 1.0
end

local function prepareFinalAlignment()
  local target,yaw=final_heading.endpoints(mission:num_commands(),function(i) return mission:get_item(i) end)
  if not target then
    gcs:send_text(MAV_SEVERITY.WARNING,'Final heading: mission geometry unavailable')
    final_state='finished'; return
  end
  final_target=target; final_yaw=yaw; final_state='waiting'; final_ticks=0
end

local function releaseControlAllocation()
  if outputs_overridden then
    for _, output in ipairs(motor_outputs) do
      -- timeout=0 hands the channel back to the native motor driver.
      SRV_Channels:set_output_pwm_chan_timeout(output.channel, output.trim, 0)
    end
    outputs_overridden = false
  end
end

local function loadMotorOutputs()
  releaseControlAllocation()
  outputs_loaded = false
  local pwm_type = param:get('MOT_PWM_TYPE')
  if pwm_type ~= nil and pwm_type ~= 0 then
    gcs:send_text(MAV_SEVERITY.ERROR, 'Allocation: requires normal bidirectional PWM (MOT_PWM_TYPE=0)')
    return false
  end
  if (param:get('WRC_ENABLE') or 0) ~= 0 then
    gcs:send_text(MAV_SEVERITY.ERROR, 'Allocation: wheel-rate controller requires native outputs')
    return false
  end
  local configured = {}
  local have_left, have_right = false, false
  for servo = 1, 32 do
    local prefix = string.format('SERVO%d_', servo)
    local fn = param:get(prefix .. 'FUNCTION')
    if fn == 73 or fn == 74 then
      local output = {
        channel = servo - 1, left = fn == 73,
        minimum = param:get(prefix .. 'MIN'), trim = param:get(prefix .. 'TRIM'),
        maximum = param:get(prefix .. 'MAX'), reversed = param:get(prefix .. 'REVERSED') == 1
      }
      if not funcs:normalizedToPwm(0, output.minimum, output.trim, output.maximum, false) then
        gcs:send_text(MAV_SEVERITY.ERROR, 'Allocation: invalid limits for ' .. prefix)
        return false
      end
      configured[#configured + 1] = output
      have_left = have_left or output.left
      have_right = have_right or not output.left
    end
  end
  motor_outputs = configured
  outputs_loaded = have_left and have_right
  return outputs_loaded
end

local function applyControlAllocation(t, s)
  local left, right = funcs:allocateRightAndLeft(t, s,
    param:get('MOT_STR_THR_MIX'), param:get('MOT_THST_ASYM'))
  local minimum, expo = param:get('MOT_THR_MIN'), param:get('MOT_THST_EXPO')
  left = funcs:scaleMotorOutput(left, minimum, expo)
  right = funcs:scaleMotorOutput(right, minimum, expo)
  for _, output in ipairs(motor_outputs) do
    local command = output.left and left or right
    local pwm = funcs:normalizedToPwm(command, output.minimum, output.trim, output.maximum, output.reversed)
    SRV_Channels:set_output_pwm_chan_timeout(output.channel, pwm, OUTPUT_TIMEOUT_MS)
  end
  outputs_overridden = true
end

local function applyAutopilotControl()
  -- The autopilot has already applied RCMAP, RC reversal, trim and deadzone.
  -- Keep signed commands, including stop, reverse and active braking.
  applyControlAllocation(vehicle:get_control_output(THROTTLE_CONTROL_OUTPUT_CHANNEL),
    vehicle:get_control_output(CONTROL_OUTPUT_YAW))
end

-- A rota e o desvio continuam calculados pelo ArduRover; so no fim da missao
-- alinhamos o rover ao segmento dos dois ultimos waypoints.
local function updateFinalAlignment()
  if final_state=='finished' then return false end
  if final_state=='idle' then prepareFinalAlignment() end
  if final_state=='finished' then return false end
  local loc,velocity,gyro,yaw=ahrs:get_position(),ahrs:groundspeed_vector(),ahrs:get_gyro(),ahrs:get_yaw()
  if not ahrs:healthy() or not loc or not velocity or not gyro or not funcs:isFinite(yaw) then
    releaseControlAllocation()
    if final_state=='rotating' then
      final_state='finished'
      gcs:send_text(MAV_SEVERITY.WARNING,'Final heading: stopped, navigation data unavailable')
    end
    return true
  end
  local ground_speed, yaw_rate=velocity:length(),gyro:z()
  if not funcs:isFinite(ground_speed) or not funcs:isFinite(yaw_rate) then
    releaseControlAllocation()
    if final_state=='rotating' then final_state='finished' end
    return true
  end
  local distance=final_heading.distance_and_bearing(loc:lat()/1e7,loc:lng()/1e7,final_target.lat,final_target.lon)
  local max_distance=math.max(0.5,(param:get('WP_RADIUS') or 0)+0.25)
  if not distance or distance>max_distance or not nearbyClear() then
    releaseControlAllocation()
    if final_state=='rotating' then
      gcs:send_text(MAV_SEVERITY.WARNING,'Final heading: stopped, position or obstacle check failed')
      final_state='finished'
    elseif not final_wait_warned then
      gcs:send_text(MAV_SEVERITY.WARNING,'Final heading: awaiting position and clear proximity')
      final_wait_warned=true
    end
    return true
  end
  final_wait_warned=false
  if final_state=='waiting' then
    if ground_speed>.10 or math.abs(yaw_rate)>math.rad(5) then
      releaseControlAllocation(); return true
    end
    final_state='rotating'
    gcs:send_text(MAV_SEVERITY.NOTICE,'Final heading: aligning to last segment')
  end
  final_ticks=final_ticks+1
  if final_ticks>1250 then
    releaseControlAllocation(); final_state='finished'
    gcs:send_text(MAV_SEVERITY.WARNING,'Final heading: timeout')
    return true
  end
  local steer,error=final_heading.steer(yaw,final_yaw,yaw_rate)
  if steer==nil then releaseControlAllocation(); final_state='finished'; return true end
  if math.abs(error)<=math.rad(5) and math.abs(yaw_rate)<=math.rad(5) then
    aligned_ticks=aligned_ticks+1
  else
    aligned_ticks=0
  end
  if aligned_ticks>=50 then
    releaseControlAllocation();final_state='finished'
    gcs:send_text(MAV_SEVERITY.NOTICE,'Final heading: aligned')
    return true
  end
  applyControlAllocation(0,steer)
  return true
end

-- HOLD e desarmamento mantem a autoridade do controlador nativo.
local function update()
  if param:get('SCR_USER5') ~= 2 then
    releaseControlAllocation()
    if not vehicle_type_warning then
      gcs:send_text(MAV_SEVERITY.WARNING, 'Allocation inactive: waiting for SCR_USER5=2')
      vehicle_type_warning = true
    end
    return update, 1000
  end
  vehicle_type_warning = false
  if not arming:is_armed() then
    releaseControlAllocation()
    resetFinalAlignment()
    loadMotorOutputs()
    return update, 200
  end
  if SRV_Channels:get_emergency_stop() then
    releaseControlAllocation()
    return update, UPDATE_PERIOD_MS
  end
  if not allocation_bound then
    allocation_bound = allocation_param:init('BATT_SOC_ALLOC')
  end
  if not allocation_bound or allocation_param:get() ~= 1 then
    releaseControlAllocation()
    resetFinalAlignment()
    return update, UPDATE_PERIOD_MS
  end
  if vehicle:get_mode() == 4 then -- HOLD: preserve native braking
    releaseControlAllocation()
    resetFinalAlignment()
    return update, UPDATE_PERIOD_MS
  end
  if not outputs_loaded and not loadMotorOutputs() then
    return update, 1000
  end
  if vehicle:get_mode()==10 and mission:state()==mission.MISSION_COMPLETE then
    if updateFinalAlignment() then return update, UPDATE_PERIOD_MS end
  else
    resetFinalAlignment()
  end
  applyAutopilotControl()
  return update, UPDATE_PERIOD_MS
end

return update, 3000
