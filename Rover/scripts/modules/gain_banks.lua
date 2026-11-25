--[[
PWM / CAN gain banks. ATC_* and CRUISE_THROTTLE hold the set of the active
output path; the other path's copy lives in CANT_C_* (CAN) or CANT_P_* (PWM)
and is swapped in while disarmed. CANT_ACTIVE says which set is loaded.
--]]
if ... == nil then
  return
end

local R = require("rover_common")
local MAV_SEVERITY = R.MAV_SEVERITY

local GB = {}

-- Preserve the saved slots of the CAN ATC gain banks. Slots 1-3 were yaw settings.
local CAN_GAINS_PARAM_TABLE_KEY = 19
local CAN_GAIN_SWAP = {
  { 'ATC_STR_RAT_FF', 'STRFF' },
  { 'ATC_STR_RAT_P', 'STRP' },
  { 'ATC_STR_RAT_I', 'STRI' },
  { 'ATC_SPEED_P', 'SPDP' },
  { 'ATC_SPEED_I', 'SPDI' },
  { 'CRUISE_THROTTLE', 'CRUISE' },
}
assert(param:add_table(CAN_GAINS_PARAM_TABLE_KEY, 'CANT_', 5 + 2 * #CAN_GAIN_SWAP), 'could not add CANT_ parameter table')
assert(param:add_param(CAN_GAINS_PARAM_TABLE_KEY, 4, 'ACTIVE', 0), 'could not add CANT_ACTIVE parameter')
assert(param:add_param(CAN_GAINS_PARAM_TABLE_KEY, 5, 'C_VALID', 0), 'could not add CANT_C_VALID parameter')
for n, item in ipairs(CAN_GAIN_SWAP) do
  assert(param:add_param(CAN_GAINS_PARAM_TABLE_KEY, 5 + n, 'C_' .. item[2], 0), 'could not add CANT_C_' .. item[2])
end
for n, item in ipairs(CAN_GAIN_SWAP) do
  assert(param:add_param(CAN_GAINS_PARAM_TABLE_KEY, 5 + #CAN_GAIN_SWAP + n, 'P_' .. item[2], 0), 'could not add CANT_P_' .. item[2])
end

local function setAndSaveIfChanged(name, value)
  if value ~= nil and param:get(name) ~= value then
    param:set_and_save(name, value)
  end
end

--[[
Keep ATC_* / CRUISE_THROTTLE on the gain set of the active output path.
Only called while disarmed. With PWM only (CANT_ACTIVE = 0 and CAN never on)
nothing is written.
--]]
function GB.sync()
  -- Give the CAN driver time to report the path after boot before deciding
  if millis():toint() < 6000 then
    return
  end
  local can = R.can_selected()
  local want = can and 1 or 0
  local active = param:get('CANT_ACTIVE') or 0
  if want == active then
    if can then
      -- QuickTune and the Circle relay write ATC_*; keep the CAN copy current
      for _, item in ipairs(CAN_GAIN_SWAP) do
        setAndSaveIfChanged('CANT_C_' .. item[2], param:get(item[1]))
      end
    end
  else
    local out_prefix = (active == 1) and 'CANT_C_' or 'CANT_P_'
    for _, item in ipairs(CAN_GAIN_SWAP) do
      setAndSaveIfChanged(out_prefix .. item[2], param:get(item[1]))
    end
    if can and (param:get('CANT_C_VALID') or 0) == 0 then
      for _, item in ipairs(CAN_GAIN_SWAP) do
        setAndSaveIfChanged('CANT_C_' .. item[2], param:get(item[1]))
      end
      param:set_and_save('CANT_C_VALID', 1)
      gcs:send_text(MAV_SEVERITY.WARNING, 'CAN gains: first CAN use, starting from the PWM gains; retune in CAN')
    else
      local in_prefix = can and 'CANT_C_' or 'CANT_P_'
      for _, item in ipairs(CAN_GAIN_SWAP) do
        setAndSaveIfChanged(item[1], param:get(in_prefix .. item[2]))
      end
    end
    param:set_and_save('CANT_ACTIVE', want)
    gcs:send_text(MAV_SEVERITY.NOTICE, string.format('Gain set: %s loaded into ATC_*', can and 'CAN' or 'PWM'))
  end
end

return GB
