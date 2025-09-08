-- Check if the script is being run directly by APM, if so exit
if ... == nil then
  return
end

local funcs = {}
funcs.__index = funcs

-------------------------------------------------------------------------------
-- General math section
function funcs:toRadians(mdegrees)
  if mdegrees == nil then
    return 0
  end
  return mdegrees * math.pi / 180.0
end

function funcs:toDegrees(mradians)
  if mradians == nil then
    return 0
  end
  return mradians * 180.0 / math.pi
end

function funcs:dotProduct(u_x, u_y, v_x, v_y)
  return u_x * v_x + u_y * v_y
end

-- Limiting a value to a range
function funcs:mapMaxMin(value, min, max)
  if value > min and value < max then
    return value
  elseif value < min then
    return min
  else
    return max
  end
end

-------------------------------------------------------------------------------

-- Making sure the error is within the range [-180, 180]
-- Check for the loop to bring back into the range properly
function funcs:mapErrorToRange(input_error)
  if input_error > -180 and input_error < 180 then
    return input_error
  elseif input_error < -180 then
    return input_error + 360
  else
    return input_error - 360
  end
end

-- Function to map an angle to the range [0, 360]
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
--]]
function funcs:calculateBearingBetweenPoints(lat_1, lon_1, lat_2, lon_2)
  local rad_lat_1, rad_lon_1 = funcs:toRadians(lat_1), funcs:toRadians(lon_1)
  local rad_lat_2, rad_lon_2 = funcs:toRadians(lat_2), funcs:toRadians(lon_2)
  local d_lon = rad_lon_2 - rad_lon_1
  local y = math.sin(d_lon) * math.cos(rad_lat_2)
  local x = math.cos(rad_lat_1) * math.sin(rad_lat_2) - math.sin(rad_lat_1) * math.cos(rad_lat_2) * math.cos(d_lon)
  local bearing = math.atan(y, x)
  -- Transform to match the ardupilot convention and map to [0, 360]
  bearing = funcs:toDegrees(math.pi / 2 - bearing)
  return (bearing + 360) % 360
end

-- Calculate distance (latlon to meters)
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

-- Returns the point projected in a line, with some advance not to be so strict
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

function funcs:applyAbsSmoothing(current_value, last_value, max_change_rate, transition_rate)
  -- Test if we are requiring a change in signal greater than the max allowed
  local output_value = current_value
  if math.abs(last_value - current_value) > max_change_rate then
    -- If transition is high, apply the transition rate to the output signal
    output_value = last_value + (output_value - last_value) * transition_rate
  end
  return output_value
end

function funcs:applyDeadZone(value, trim, dead_zone)
  -- Check if the values are within the dead zone to return TRIM
  if value > trim - dead_zone and value < trim + dead_zone then
    return math.floor(trim)
  end
  return math.floor(value)
end

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

-- Calculates the cross-track error using vehicle and projection coordinates.
--
-- @param velocity The vehicle's current velocity.
-- @param gain The gain constant for the cross-track correction.
-- @param p_x The x-coordinate of the projection point on the path.
-- @param p_y The y-coordinate of the projection point on the path.
-- @param vh_x The vehicle's x-coordinate.
-- @param vh_y The vehicle's y-coordinate.
-- @return The cross-track error.
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

-- Determines if a vehicle is to the left or right of a line segment.
--
-- @param start_x The lon coordinate of the start point.
-- @param start_y The lat coordinate of the start point.
-- @param end_x The lon coordinate of the end point.
-- @param end_y The lat coordinate of the end point.
-- @param vehicle_x The lon coordinate of the vehicle.
-- @param vehicle_y The lat coordinate of the vehicle.
-- @return 1 if left, -1 if right
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
