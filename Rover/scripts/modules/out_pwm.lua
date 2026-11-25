--[[
PWM output path: the mixer's throttle / steering as servo pulses on MAIN 1..4.

  1 - 0     ^
    |       | Rover forward direction
  2 - 3
Left motors are 1 and 2, right motors 0 and 3.

Remove this file to run on CAN only. The mixer then writes no PWM at all.
THRO_MAX_LUA / THRO_MIN_LUA live in the mixer's THRO_ table (R.thro_max_lua,
R.thro_min_lua); THRO_MIN_LUA is the minimum pulse of the PWM yaw kick.
--]]
if ... == nil then
  return
end

local R = require("rover_common")
local funcs = require("functions")
local MAV_SEVERITY = R.MAV_SEVERITY
local PWM_TRIM_VALUE = R.PWM_TRIM_VALUE

local PWM = {}

-- Half-range of the pulse around trim, us (THRO_MAX_LUA % of 500 us)
function PWM.range()
  return 5 * math.max(0, math.min(100, R.thro_max_lua:get()))
end

-- Left / right pulses for throttle t and steering s (-1..1)
function PWM.allocate(t, s)
  local pwm_range = PWM.range()
  local pwm_aloc_l, pwm_aloc_r = funcs:allocateRightAndLeftPwmShare(t, s, pwm_range)
  local pwm_min = PWM_TRIM_VALUE - pwm_range
  local pwm_max = PWM_TRIM_VALUE + pwm_range
  local pwm_l = math.floor(funcs:mapMaxMin(PWM_TRIM_VALUE + pwm_aloc_l, pwm_min, pwm_max))
  local pwm_r = math.floor(funcs:mapMaxMin(PWM_TRIM_VALUE - pwm_aloc_r, pwm_min, pwm_max))
  return pwm_l, pwm_r
end

function PWM.write(pwm_l, pwm_r, timeout_ms)
  SRV_Channels:set_output_pwm_chan_timeout(0, pwm_r, timeout_ms)
  SRV_Channels:set_output_pwm_chan_timeout(1, pwm_l, timeout_ms)
  SRV_Channels:set_output_pwm_chan_timeout(2, pwm_l, timeout_ms)
  SRV_Channels:set_output_pwm_chan_timeout(3, pwm_r, timeout_ms)
end

-- Disarmed: hold every motor at trim
function PWM.trim()
  SRV_Channels:set_output_pwm_chan_timeout(0, PWM_TRIM_VALUE, 3000)
  SRV_Channels:set_output_pwm_chan_timeout(1, PWM_TRIM_VALUE, 3000)
  SRV_Channels:set_output_pwm_chan_timeout(2, PWM_TRIM_VALUE, 3000)
  SRV_Channels:set_output_pwm_chan_timeout(3, PWM_TRIM_VALUE, 3000)
end

--[[
Yaw kick: the ESC gets a raw duty, so a stalled rover needs a minimum output
(THRO_MIN_LUA) to start turning. Returns s_out raised to that minimum.
--]]
function PWM.min_command(s_out)
  local pwm_range = PWM.range()
  local min_offset = R.thro_min_lua:get() - PWM_TRIM_VALUE
  if pwm_range > 0 and min_offset > 0 then
    local min_command = math.min(1, min_offset / pwm_range)
    s_out = s_out < 0 and math.min(s_out, -min_command) or math.max(s_out, min_command)
  end
  return s_out
end

--[[
Save the measured turn breakaway as THRO_MIN_LUA, midway between the last
relay output that left the rover still and the first one that turned it.
--]]
function PWM.save_breakaway(still, move)
  local lvl = (still + move) / 2
  local pwm = math.floor(PWM_TRIM_VALUE + lvl * PWM.range() + 0.5)
  pwm = math.max(PWM_TRIM_VALUE, math.min(2000, pwm))
  param:set_and_save('THRO_MIN_LUA', pwm)
  gcs:send_text(MAV_SEVERITY.NOTICE, string.format("YawTune: breakaway %.0f%% (still %.0f, turns %.0f); THRO_MIN_LUA=%d",
    lvl * 100, still * 100, move * 100, pwm))
end

R.pwm = PWM
return PWM
