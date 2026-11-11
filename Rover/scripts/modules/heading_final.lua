-- Geometria da orientacao final de uma missao, sem alterar os waypoints.
local M = {}
local R = 6378137
local rad = math.rad
local atan2 = math.atan2 or math.atan

local function finite(x)
  return type(x) == 'number' and x == x and math.abs(x) < math.huge
end

function M.distance_and_bearing(lat1, lon1, lat2, lon2)
  if not (finite(lat1) and finite(lon1) and finite(lat2) and finite(lon2)) then return nil end
  if math.abs(lat1) > 90 or math.abs(lat2) > 90 or math.abs(lon1) > 180 or math.abs(lon2) > 180 then return nil end
  local north = rad(lat2-lat1)*R
  local dl = (lon2-lon1+180)%360-180
  local east = rad(dl)*R*math.cos(rad((lat1+lat2)*.5))
  local distance=math.sqrt(north*north+east*east)
  return distance, atan2(east,north)
end

-- Apenas termina em waypoint geografico; nao supoe a ordem real em missoes com saltos.
function M.endpoints(count, get_item)
  if type(count) ~= 'number' or count < 3 or count > 1000 then return nil end
  local previous, last = nil, nil
  for i=1,count-1 do
    local item=get_item(i)
    if not item then return nil end
    local command=item:command()
    if command==177 or command==600 or command==601 then return nil end
    if command==16 then
      local frame=item:frame()
      if frame~=0 and frame~=3 and frame~=5 and frame~=6 and frame~=10 and frame~=11 then return nil end
      local lat,lon=item:x()/1e7,item:y()/1e7
      if not (finite(lat) and finite(lon)) or (lat==0 and lon==0) then return nil end
      previous=last
      last={lat=lat,lon=lon,index=i}
    elseif command>=16 and command<=95 and command~=92 then
      -- Outro comando NAV depois de um waypoint invalida a orientacao final.
      previous,last=nil,nil
    end
  end
  if not previous or not last then return nil end
  local distance,heading=M.distance_and_bearing(previous.lat,previous.lon,last.lat,last.lon)
  if not distance or distance<.5 then return nil end
  return last,heading
end

function M.steer(yaw,target,gyro_z)
  if not (finite(yaw) and finite(target) and finite(gyro_z)) then return nil end
  local error=(target-yaw+math.pi)%(2*math.pi)-math.pi
  if math.abs(error)<=rad(5) then return 0,error end
  local output=math.max(-.22,math.min(.22,error*.4-gyro_z*.12))
  if math.abs(output)<.10 then output=error>0 and .10 or -.10 end
  return output,error
end
return M
