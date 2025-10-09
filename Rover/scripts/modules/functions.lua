-- Check if the script is being run directly by APM, if so exit
if ... == nil then
  return
end

local funcs = {}
funcs.__index = funcs

--[[
Converts degrees to radians
-- @param mdegrees number - Angle in degrees
-- @return number - Angle in radians
--]]
function funcs:toRadians(mdegrees)
  if mdegrees == nil then
    return 0
  end
  return mdegrees * math.pi / 180.0
end

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
Dot product of two 2D vectors
-- @param u_x number - X component of the first vector
-- @param u_y number - Y component of the first vector
-- @param v_x number - X component of the second vector
-- @param v_y number - Y component of the second vector
-- @return number - The dot product of the two vectors
--]]
function funcs:dotProduct(u_x, u_y, v_x, v_y)
  return u_x * v_x + u_y * v_y
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
Making sure the error is within the range [-180, 180]
-- @param input_error number - The input error value
-- @return number - The mapped error value within the range [-180, 180]
--]]
function funcs:mapErrorToRange(input_error)
  if input_error > -180 and input_error < 180 then
    return input_error
  elseif input_error < -180 then
    return input_error + 360
  else
    return input_error - 360
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
Calculates orientation based on two pairs of geographic coordinates (the vector from one to the other)
First point is the origin, second on is the target of our orientation.
0 degrees should be north, 90 degrees east
-- @param lat_1 number - Latitude of the first point in degrees
-- @param lon_1 number - Longitude of the first point in degrees
-- @param lat_2 number - Latitude of the second point in degrees
-- @param lon_2 number - Longitude of the second point in degrees
-- @return number - Bearing in degrees from the first point to the second point [0, 360]
--]]
function funcs:calculateBearingBetweenPoints(lat_1, lon_1, lat_2, lon_2)
  local rad_lat_1, rad_lon_1 = funcs:toRadians(lat_1), funcs:toRadians(lon_1)
  local rad_lat_2, rad_lon_2 = funcs:toRadians(lat_2), funcs:toRadians(lon_2)
  local d_lon = rad_lon_2 - rad_lon_1
  local y = math.sin(d_lon) * math.cos(rad_lat_2)
  local x = math.cos(rad_lat_1) * math.sin(rad_lat_2) - math.sin(rad_lat_1) * math.cos(rad_lat_2) * math.cos(d_lon)
  local bearing = math.atan(y, x)
  -- Transform to match the ardupilot convention and map to [0, 360]
  bearing = funcs:toDegrees(bearing)
  return (bearing + 360) % 360
end

--[[
Calculate distance (latlon to meters) using the Haversine formula
-- @param lat_1 number - Latitude of the first point in degrees
-- @param lon_1 number - Longitude of the first point in degrees
-- @param lat_2 number - Latitude of the second point in degrees
-- @param lon_2 number - Longitude of the second point in degrees
-- @return number - Distance between the two points in meters
--]]
function funcs:haversineDistance(lat_1, lon_1, lat_2, lon_2)
  local R = 6371000 -- Earth radius [m]
  local rad_lat_1, rad_lon_1 = funcs:toRadians(lat_1), funcs:toRadians(lon_1)
  local rad_lat_2, rad_lon_2 = funcs:toRadians(lat_2), funcs:toRadians(lon_2)
  local deltaLat = rad_lat_2 - rad_lat_1
  local deltaLon = rad_lon_2 - rad_lon_1

  local a = math.sin(deltaLat / 2) ^ 2 +
      math.cos(rad_lat_1) * math.cos(rad_lat_2) *
      math.sin(deltaLon / 2) ^ 2

  local c = 2 * math.atan(math.sqrt(a), math.sqrt(1 - a))

  return R * c
end

--[[
Project a point (p) onto a line defined by start (s) and end (e) points
-- @param p_x number - X coordinate of the point to be projected
-- @param p_y number - Y coordinate of the point to be projected
-- @param s_x number - X coordinate of the start point of the line
-- @param s_y number - Y coordinate of the start point of the line
-- @param e_x number - X coordinate of the end point of the line
-- @param e_y number - Y coordinate of the end point of the line
-- @return number, number - The projected point coordinates (x, y)
--]]
function funcs:lineProjectionPoint(p_x, p_y, s_x, s_y, e_x, e_y)
  -- Calculate the projection scalar t of our current (p)
  -- on top of the line formed from start (s) to end (e) points
  local sp_x, sp_y = p_x - s_x, p_y - s_y
  local se_x, se_y = e_x - s_x, e_y - s_y
  local se_length_squared = se_x ^ 2 + se_y ^ 2
  local t = funcs:dotProduct(sp_x, sp_y, se_x, se_y) / se_length_squared

  -- Return the projected point coordinates based on the scaler t
  return s_x + t * se_x, s_y + t * se_y
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

--[[
Calculates the cross-track error using vehicle and projection coordinates.
-- @param velocity The vehicle's current velocity.
-- @param gain The gain constant for the cross-track correction.
-- @param p_x The x-coordinate of the projection point on the path.
-- @param p_y The y-coordinate of the projection point on the path.
-- @param vh_x The vehicle's x-coordinate.
-- @param vh_y The vehicle's y-coordinate.
-- @return The cross-track error.
--]]
function funcs:crossTrackError(velocity, gain, p_x, p_y, vh_x, vh_y)
    local cross_track_error = funcs:haversineDistance(vh_x, vh_y, p_x, p_y)
    if cross_track_error == nil then
      return 0
    end
    if velocity == nil or velocity < 0.1 then
      return 0
    end
    -- Calculate the steering correction based on the cross-track error
    local steering_correction = gain * cross_track_error / velocity
    return steering_correction
end

--[[
Determines if a vehicle is to the left or right of a line segment.
-- @param start_x The lon coordinate of the start point.
-- @param start_y The lat coordinate of the start point.
-- @param end_x The lon coordinate of the end point.
-- @param end_y The lat coordinate of the end point.
-- @param vehicle_x The lon coordinate of the vehicle.
-- @param vehicle_y The lat coordinate of the vehicle.
-- @return 1 if left, -1 if right
--]]
function funcs:lineSideSignal(start_x, start_y, end_x, end_y, vehicle_x, vehicle_y)
    -- Vectors
    local dx1 = end_x - start_x
    local dy1 = end_y - start_y
    local dx2 = vehicle_x - start_x
    local dy2 = vehicle_y - start_y
    -- Cross product
    local cross = dx1 * dy2 - dy1 * dx2
    if cross > 0 then
        return 1   -- left
    elseif cross < 0 then
        return -1  -- right
    else
        return 0   -- collinear
    end
end

return funcs
