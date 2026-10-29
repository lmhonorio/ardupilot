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
  assert(self:isFinite(value) and self:isFinite(min) and self:isFinite(max) and min <= max,
    "mapMaxMin requires finite values and ordered limits")
  return math.min(math.max(value, min), max)
end

--[[
Function to map an angle to the range [0, 360]
-- @param angle number - The input angle in degrees
-- @return number - The mapped angle in the range [0, 360]
--]]
function funcs:mapTo360(angle)
  if not self:isFinite(angle) then
    return 0
  end
  return angle % 360
end

--[[
Detect NaN (in Lua: NaN ~= NaN)
-- @param x number
-- @return boolean
--]]
function funcs:isNan(x)
  return x ~= x
end

function funcs:isFinite(value)
  return type(value) == "number" and value == value and math.abs(value) < math.huge
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
Compatibility wrapper returning signed PWM offsets for each side
-- @param t number - Signed throttle command from -1 to 1
-- @param s number - Steering command from -1.0 to 1.0
-- @param pwm_range number - Maximum PWM range for the motors
-- @return number, number - The allocated PWM values for the left and right motors
--]]
function funcs:allocateRightAndLeftPwmShare(t, s, pwm_range)
  if not self:isFinite(pwm_range) or pwm_range <= 0 then
    return 0, 0
  end
  local left, right = self:allocateRightAndLeft(t, s)
  local function rounded(value)
    local magnitude = math.floor(math.abs(value * pwm_range) + 0.5)
    return value < 0 and -magnitude or magnitude
  end
  return rounded(left), rounded(right)
end

-- Positive steering makes the left side faster: right turn, also in reverse.
-- Match AP_MotorsUGV saturation priorities and forward/reverse thrust limits.
-- Defaults retain fair mixing for symmetric motors.
function funcs:allocateRightAndLeft(t, s, mix, asymmetry)
  if not self:isFinite(t) or not self:isFinite(s) then
    return 0, 0
  end
  t = self:mapMaxMin(t, -1, 1)
  s = self:mapMaxMin(s, -1, 1)
  mix = self:isFinite(mix) and self:mapMaxMin(mix, 0, 1) or 0.5
  asymmetry = self:isFinite(asymmetry) and math.max(asymmetry, 1) or 1
  local lower = -1 / asymmetry
  local best = (1 + lower) / 2
  local steering_range = t < best and (math.max(t, 0) - lower) or (1 - best)
  s = self:mapMaxMin(s, -steering_range, steering_range)
  t = self:mapMaxMin(t, lower, 1)
  local saturation = math.max(t + math.abs(s), (t - math.abs(s)) / lower)
  if saturation > 1 then
    local fair = 1 / saturation
    if mix >= 0.5 then
      s = s * (fair + (1 - fair) * (2 * mix - 1))
      t = t >= best and (1 - math.abs(s)) or (math.abs(s) + lower)
    else
      t = t * (fair + (1 - fair) * (1 - 2 * mix))
      local direction = s > 0 and 1 or -1
      s = (t >= best and (1 - t) or (t - lower)) * direction
    end
  end
  local left, right = t + s, t - s
  if left < 0 then left = left * asymmetry end
  if right < 0 then right = right * asymmetry end
  return self:mapMaxMin(left, -1, 1), self:mapMaxMin(right, -1, 1)
end

-- Match AP_MotorsUGV::get_scaled_throttle, keeping zero exactly neutral.
function funcs:scaleMotorOutput(value, minimum_percent, expo)
  if not self:isFinite(value) or value == 0 then return 0 end
  local direction = value < 0 and -1 or 1
  local minimum = self:isFinite(minimum_percent) and self:mapMaxMin(minimum_percent / 100, 0, 1) or 0
  local magnitude = minimum + math.min(math.abs(value), 1) * (1 - minimum)
  if self:isFinite(expo) and expo ~= 0 and math.abs(expo) <= 1 then
    magnitude = ((expo - 1) + math.sqrt(math.max(0, (1 - expo)^2 + 4 * expo * magnitude))) / (2 * expo)
  end
  return direction * magnitude
end

-- Map zero to this channel's trim, using independent forward/reverse spans.
-- Raw PWM overrides bypass SERVOx_REVERSED; apply reversal exactly once.
function funcs:normalizedToPwm(value, minimum, trim, maximum, reversed)
  if not self:isFinite(minimum) or not self:isFinite(trim) or not self:isFinite(maximum)
      or not (0 <= minimum and minimum < trim and trim < maximum and maximum <= 65535) then
    return nil
  end
  if not self:isFinite(value) then
    value = 0
  end
  value = self:mapMaxMin(value, -1, 1)
  if reversed then
    value = -value
  end
  local span = value >= 0 and (maximum - trim) or (trim - minimum)
  return math.floor(self:mapMaxMin(trim + value * span, minimum, maximum) + 0.5)
end

return funcs
