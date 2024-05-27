-- Check if the script is being run directly by APM, if so exit
if ... == nil then
    return
end

local funcs = {}
funcs.__index = funcs

function funcs:MapToUnid(value)
    if value < -1 then
        return  -1
    elseif value > 1 then
      return 1
    else
        return value
    end
  end

-- Calculate the resulting value
function funcs:map_error(resulting)
    if resulting == nil then
        return 0
    end

    if resulting == 0 then
        return 0
    elseif math.abs(resulting) < 180 then
        return -resulting
    else
        return (math.abs(resulting)/(resulting + 0.001))*(360 - math.abs(resulting))
    end
end

-- Function to map an angle to the range [0, 360]
function funcs:map_to_360(angle)
    if angle == nil then
        return 0
    end

    local mapped_angle = angle
    if mapped_angle < 0 then
        mapped_angle = mapped_angle + 360
    end
    
    return mapped_angle
end

-- Conversion between degrees and radians
function funcs:to_radians(mdegrees)
    if mdegrees == nil then
        return 0
    end
    return mdegrees*math.pi/180.0
end

function funcs:to_degrees(mradians)
    if mradians == nil then
        return 0
    end
    return mradians*180.0/math.pi
end


  
  function funcs:Vector_magnitude(x, y)
    return math.sqrt(x^2 + y^2)
  end
  
  function funcs:Dot_product(u_x, u_y, v_x, v_y)
    return u_x * v_x + u_y * v_y
  end
  
  function funcs:Calculate_angle(u_x, u_y, v_x, v_y)
    local dot = funcs:Dot_product(u_x, u_y, v_x, v_y)
    local mag_u = funcs:Vector_magnitude(u_x, u_y)
    local mag_v = funcs:Vector_magnitude(v_x, v_y)
    local cos_theta = dot / (mag_u * mag_v)
    local angle = math.acos(cos_theta)  -- Resultado em radianos
    return angle
  end
  
  function  funcs:Calculate_correction_angle(p0x, p0y, p1x, p1y, rx, ry)
    local dxwp = p1x - p0x
    local dywp = p1y - p0y
    local dx_r_wp = p1x - rx
    local dy_r_wp = p1y - ry
  
    local angle = funcs:Calculate_angle(dxwp, dywp, dx_r_wp, dy_r_wp)
  
    return angle
  end
  
  
  function funcs:To_cartesian(r, theta)
    local x = r * math.cos(theta)
    local y = r * math.sin(theta)
    return x, y
  end
  
  function funcs:To_polar(x, y)
    local r = math.sqrt(x^2 + y^2)
    local theta = math.atan(y, x)
    return r, theta
  end
  
  function funcs:Add_polars(r1, theta1, r2, theta2)
    local x1, y1 = funcs:To_cartesian(r1, theta1)
    local x2, y2 = funcs:To_cartesian(r2, theta2)
  
    local x_total = x1 + x2
    local y_total = y1 + y2
  
    return funcs:To_polar(x_total, y_total)
  end
  
  -- Calcula a orientação de um vetor dado
  function Calculate_bearing(lat1, lon1, lat2, lon2)
    local radLat1, radLon1 = funcs:To_radians(lat1), funcs:To_radians(lon1)
    local radLat2, radLon2 = funcs:To_radians(lat2), funcs:To_radians(lon2)
    local dLon = radLon2 - radLon1
    local y = math.sin(dLon) * math.cos(radLat2)
    local x = math.cos(radLat1) * math.sin(radLat2) - math.sin(radLat1) * math.cos(radLat2) * math.cos(dLon)
    local bearing = math.atan(y, x)
    return (funcs:To_degrees(bearing) + 360) % 360  -- Normaliza o resultado para (0, 360)
  end
  
  -- Função Haversine para calcular distância entre dois pontos geográficos
  function funcs:Haversine_distance(lat1, lon1, lat2, lon2)
    local R = 6371000 -- Raio da Terra em metros
    local radLat1, radLon1 = funcs:To_radians(lat1), funcs:To_radians(lon1)
    local radLat2, radLon2 = funcs:To_radians(lat2), funcs:To_radians(lon2)
    local deltaLat = radLat2 - radLat1
    local deltaLon = radLon2 - radLon1
  
    local a = math.sin(deltaLat / 2) ^ 2 + 
              math.cos(radLat1) * math.cos(radLat2) * 
              math.sin(deltaLon / 2) ^ 2
  
    local c = 2 * math.atan(math.sqrt(a), math.sqrt(1 - a))
  
    return R * c
  end
  
  
  function funcs:Point_to_line_distance(px, py, psi, x0, y0, x1, y1)
    local dx, dy = x1 - x0, y1 - y0
    local length_squared = dx^2 + dy^2
    local t = ((px - x0) * dx + (py - y0) * dy) / length_squared
  
    -- Se a projeção cai fora do segmento de reta, o mais próximo é o ponto final mais próximo
      local nearest_x, nearest_y
      if t < 0 then
          nearest_x, nearest_y = x0, y0
      elseif t > 1 then
          nearest_x, nearest_y = x1, y1
      else
          nearest_x = x0 + t * dx
          nearest_y = y0 + t * dy
      end
  
    local distancia = funcs:Haversine_distance(px, py, nearest_x, nearest_y)
    local bearing_to_wp = funcs:Calculate_bearing(px, py, x1, y1)
    --local angle_difference = (bearing_to_wp - psi + 360) % 360
    local angle_difference = (bearing_to_wp - psi/2 + 360) % 360
  
    local orientacao = funcs:Point_relative_to_vector(x0,y0,x1,y1,px,py)
  
    return distancia, orientacao*angle_difference
  
  
  end

  function funcs:Point_relative_to_vector(p0x, p0y, p1x, p1y, rx, ry)
    -- Calcula as componentes do vetor AB
    local vx = p1x - p0x
    local vy = p1y - p0y
  
    -- Calcula o produto vetorial em 2D
    local cross_product = vx * (ry - p0y) - vy * (rx - p0x)
  
    -- Interpreta o resultado do produto vetorial
    if cross_product > 0 then
        return 1  -- O ponto está à esquerda do vetor
    elseif cross_product < 0 then
        return -1 -- O ponto está à direita do vetor
    else
        return 0  -- O ponto está na linha do vetor
    end
  end
  





return funcs
