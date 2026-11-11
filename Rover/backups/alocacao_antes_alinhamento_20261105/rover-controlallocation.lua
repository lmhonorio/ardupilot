-- Alocacao diferencial: aplica aos motores os comandos do controlador ArduRover.
-- O FAST-LIO permanece em lidar_odometry.lua e nao interfere na atuacao.
package.path = package.path .. ';./scripts/modules/?.lua'
local funcs = require('functions')
local THROTTLE_CONTROL_OUTPUT_CHANNEL, CONTROL_OUTPUT_YAW = 3, 4
local UPDATE_PERIOD_MS, OUTPUT_TIMEOUT_MS = 20, 100
local MAV_SEVERITY = {ERROR=3, WARNING=4}
local motor_outputs, outputs_overridden, outputs_loaded = {}, false, false
local vehicle_type_warning = false
local allocation_param, allocation_bound = Parameter(), false

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

-- HOLD e desarmamento mantem a autoridade do controlador nativo.
-- Nao altera modo, missao, rumo, obstaculos ou a referencia do controlador.
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
    return update, UPDATE_PERIOD_MS
  end
  if vehicle:get_mode() == 4 then -- HOLD: preserve native braking
    releaseControlAllocation()
    return update, UPDATE_PERIOD_MS
  end
  if not outputs_loaded and not loadMotorOutputs() then
    return update, 1000
  end
  applyAutopilotControl()
  return update, UPDATE_PERIOD_MS
end

return update, 3000
