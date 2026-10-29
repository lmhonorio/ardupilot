-- Unit test the companion packet contract, sender filtering and stale data.
local clock = 0
local ts = {}
ts.__index = {toint = function(self) return self.value end}
ts.__sub = function(a,b) return setmetatable({value=a.value-b.value},ts) end
local function time(value) return setmetatable({value=value},ts) end
millis = function() return time(clock) end
local queue, published = {}, {}
mavlink = {
  init = function() end,
  register_rx_msgid = function(_, id) assert(id == 171); return true end,
  receive_chan = function() local m=table.remove(queue,1); if m then return m,0,time(clock) end end,
}
gcs = {send_text=function() end, send_named_float=function(_,name,value) published[name]=value end}
local update = assert(loadfile('Rover/scripts/lidar_odometry.lua'))()
local function packet(sysid, sequence, marker)
  return string.pack('<I2BBBBBBBI3BBc4I4I4I8fffffffB', 0, 0xfd, 51, 0, 0, 1,
                     sysid, 197, 171, 201, 49, 'FLIO', 17, sequence, 123456,
                     1.25, -2.5, .75, 1, 0, 0, 0, marker)
end
queue[1] = packet(244, 1, 165)
update()
assert(get_lidar_odometry() == nil, 'wrong system ID accepted')
queue[1] = packet(245, 1, 165)
update()
local pose = assert(get_lidar_odometry())
assert(pose.x == 1.25 and pose.y == -2.5 and pose.z == .75 and pose.sequence == 1)
assert(published.LIO_X == 1.25 and published.LIO_Y == -2.5)
queue[1] = packet(245, 1, 165)
clock = 450; update()
clock = 501; update()
assert(get_lidar_odometry() == nil, 'duplicate should not refresh stale pose')
queue[1] = packet(245, 2, 0)
update()
assert(get_lidar_odometry() == nil, 'bad marker accepted')
queue[1] = packet(245, 2, 165)
update()
assert(get_lidar_odometry().sequence == 2, 'new pose not accepted')
print('FAST-LIO Lua receiver: filtering, units and stale timeout passed')
