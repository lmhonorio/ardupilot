-- Check if the script is being run directly by APM, if so exit
if ... == nil then
  return
end

local funcs = {}
funcs.__index = funcs

--[[
Converts radians to degrees
-- @param mradians number - Angle in radians
-- @return number - Angle in degrees
--]]
function funcs:toDegrees(mradians)
  if mradians == nil then
    return 0
  end
  return mradians * 180.0 / math.pi
end

--[[
Limits a value to be within a specified range
-- @param value number - Value to be limited
-- @param min number - Minimum limit
-- @param max number - Maximum limit
-- @return number - Limited value
--]]
function funcs:mapMaxMin(value, min, max)
  if value > min and value < max then
    return value
  elseif value < min then
    return min
  else
    return max
  end
end

--[[
Function to map an angle to the range [0, 360]
-- @param angle number - The input angle in degrees
-- @return number - The mapped angle in the range [0, 360]
--]]
function funcs:mapTo360(angle)
  if angle == nil then
    return 0
  end
  if angle < 0 then
    return angle + 360
  end
  return angle
end

--[[
Detect NaN (in Lua: NaN ~= NaN)
-- @param x number
-- @return boolean
--]]
function funcs:isNan(x)
  return x ~= x
end

--[[
Yaw error in radians: (target - current) wrapped to [-pi, pi)
-- @param current_rad number
-- @param target_rad number
-- @return number
--]]
function funcs:yawErrorRad(current_rad, target_rad)
  if current_rad == nil or target_rad == nil then
    return 0
  end
  -- Wrap diffenrence between -pi and pi
  local rad = target_rad - current_rad
  return (rad + math.pi) % (2 * math.pi) - math.pi
end

--[[
Controls the output in manual mode to avoid sudden changes
-- @param current_value number - The current input value
-- @param last_value number - The last output value
-- @param max_change_rate number - Maximum allowed change rate
-- @param transition_rate number - Transition rate to apply when exceeding max change
-- @return number - The smoothed output value
--]]
function funcs:applyAbsSmoothing(current_value, last_value, max_change_rate, transition_rate)
  -- Test if we are requiring a change in signal greater than the max allowed
  local output_value = current_value
  if math.abs(last_value - current_value) > max_change_rate then
    -- If transition is high, apply the transition rate to the output signal
    output_value = last_value + (output_value - last_value) * transition_rate
  end
  return output_value
end

--[[
Allocate the PWM signals for the right and left motors based on throttle and steering inputs
-- @param t number - Throttle command from 0 (or more) to 1
-- @param s number - Steering command from -1.0 to 1.0
-- @param pwm_range number - Maximum PWM range for the motors
-- @return number, number - The allocated PWM values for the left and right motors
--]]
function funcs:allocateRightAndLeftPwmShare(t, s, pwm_range)
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
  local pwm_aloc_l = (t_share + s_share) * pwm_range
  local pwm_aloc_r = (t_share - s_share) * pwm_range
  return math.floor(pwm_aloc_l), math.floor(pwm_aloc_r)
end

return funcs
