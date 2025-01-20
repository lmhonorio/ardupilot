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
  return (funcs:toDegrees(bearing) + 360) % 360
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

  -- Give some incresase to t, as if we were looking a bit ahead of where we are in the line
  t = t + 0.1
  if t > 0.8 then
    t = 1
  end

  -- If not 0<=t<=1, make sure we have the reference back in this range
  local projected_x, projected_y = s_x + t * se_x, s_y + t * se_y
  if t < 0 then
    projected_x, projected_y = s_x, s_y
  elseif t > 1 then
    projected_x, projected_y = e_x, e_y
  end

  return projected_x, projected_y
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
    return trim
  end
  return value
end

return funcs
