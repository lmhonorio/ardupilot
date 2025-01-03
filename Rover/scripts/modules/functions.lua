-- Check if the script is being run directly by APM, if so exit
if ... == nil then
  return
end

local funcs = {}
funcs.__index = funcs

function funcs:mapMaxMin(value, min, max)
  if value > min and value < max then
    return value
  elseif value < min then
    return min
  else
    return max
  end
end

-- Calculate the resulting value
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

function funcs:toRadians(mdegrees)
  if mdegrees == nil then
    return 0
  end
  return mdegrees*math.pi/180.0
end

function funcs:toDegrees(mradians)
  if mradians == nil then
    return 0
  end
  return mradians*180.0/math.pi
end

function funcs:dotProduct(u_x, u_y, v_x, v_y)
  return u_x * v_x + u_y * v_y
end

function funcs:toCartesian(r, theta)
  local x = r * math.cos(theta)
  local y = r * math.sin(theta)
  return x, y
end

function funcs:toPolar(x, y)
  local r = math.sqrt(x^2 + y^2)
  local theta = math.atan(y, x)
  return r, theta
end

function funcs:addPolars(r1, theta1, r2, theta2)
  local e_x, e_y = funcs:toCartesian(r1, theta1)
  local x2, y2 = funcs:toCartesian(r2, theta2)

  local x_total = e_x + x2
  local y_total = e_y + y2

  return funcs:toPolar(x_total, y_total)
end

-- Calculates orientation based on two pairs of geographic coordinates (the vector from one to the other)
function funcs:calculateBearing(lat_1, lon_1, lat_2, lon_2)
  local rad_lat_1, rad_lon_1 = funcs:toRadians(lat_1), funcs:toRadians(lon_1)
  local rad_lat_2, rad_lon_2 = funcs:toRadians(lat_2), funcs:toRadians(lon_2)
  local d_lon = rad_lon_2 - rad_lon_1
  local y = math.sin(d_lon) * math.cos(rad_lat_2)
  local x = math.cos(rad_lat_1) * math.sin(rad_lat_2) - math.sin(rad_lat_1) * math.cos(rad_lat_2) * math.cos(d_lon)
  local bearing = math.atan(y, x)
  return (funcs:toDegrees(bearing) + 360) % 360
end

-- Função Haversine para calcular distância entre dois pontos geográficos
function funcs:haversineDistance(lat_1, lon_1, lat_2, lon_2)
  local R = 6371000 -- Raio da Terra em metros
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

function funcs:lineProjectionBearing(p_x, p_y, vh_yaw, s_x, s_y, e_x, e_y)
  -- Calculate the projection scalar t of our current p on top of the line formed from start (s) to end (e) points
  local sp_x, sp_y = p_x - s_x, p_y - s_y
  local se_x, se_y = e_x - s_x, e_y - s_y
  local se_length_squared = se_x^2 + se_y^2
  local t = funcs:dotProduct(sp_x, sp_y, se_x, se_y) / se_length_squared

  -- If not 0<=t<=1, make sure we have the reference back in this range
  local projected_x, projected_y = s_x + t * se_x, s_y + t * se_y
  if t < 0 then
    projected_x, projected_y = s_x, s_y
  elseif t > 1 then
    projected_x, projected_y = e_x, e_y
  end

  local bearing_to_wp = funcs:calculateBearing(p_x, p_y, e_x, e_y)
  --local angle_difference = (bearing_to_wp - vh_yaw + 360) % 360
  local angle_difference = (bearing_to_wp - vh_yaw/2 + 360) % 360

  local side_signal = funcs:pointSideWithRespectToLine(s_x, s_y, e_x, e_y, p_x, p_y)

  return side_signal*angle_difference
end

function funcs:pointSideWithRespectToLine(p0_x, p0_y, p1_x, p1_y, r_x, r_y)
  -- Vector components
  local v_x, v_y = p1_x - p0_x, p1_y - p0_y
  -- Cross product to check the resulting vector direction
  local cross_product = v_x * (r_y - p0_y) - v_y * (r_x - p0_x)
  if cross_product > 0 then
    return 1  -- Query point to the left of 01 vector
  elseif cross_product < 0 then
    return -1 -- Query point to the right of 01 vector
  else
    return 0  -- Query point in line with 01 vector
  end
end

return funcs
