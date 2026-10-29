-- Read-only FAST-LIO pose from a companion through MAVProxy DATA64.
-- Accessible to other scripts as get_lidar_odometry(max_age_ms).
-- x/y/z are local forward/right/down metres; q is body FRD to local FRD.
-- Valid means a recent, well-formed message, not a guarantee of accuracy.
local MSG_ID, DATA_TYPE, SOURCE_SYSID, SOURCE_COMPID = 171, 201, 245, 197
local FORMAT = '<c4I4I4I8fffffffB'
local EXPECTED_LEN, MARKER = 49, 165
local TIMEOUT_MS = 500
local latest, was_fresh, last_report = nil, false, nil

local function finite(x)
  return type(x) == 'number' and x == x and math.abs(x) < math.huge
end

function get_lidar_odometry(max_age_ms)
  if latest == nil then return nil end
  local age = (millis() - latest.received_ms):toint()
  if age < 0 or age > (max_age_ms or TIMEOUT_MS) then return nil end
  return latest, age
end

LIDAR_ODOM = nil
mavlink:init(10, 4)
assert(mavlink:register_rx_msgid(MSG_ID), 'LIO DATA64 registration failed')

local function update()
  for _ = 1, 10 do
    local raw, _, received_ms = mavlink:receive_chan()
    if raw == nil then break end
    -- MAVLink message struct contains two internal bytes before the header.
    local ok, magic, len, _, _, _, sysid, compid, msgid, offset = pcall(
      string.unpack, '<BBBBBBBI3', raw, 3)
    if ok and (magic == 0xfd or magic == 0xfe) and len >= EXPECTED_LEN + 2
        and sysid == SOURCE_SYSID and compid == SOURCE_COMPID and msgid == MSG_ID then
      local subtype, data_len
      subtype, data_len, offset = string.unpack('<BB', raw, offset)
      if subtype == DATA_TYPE and data_len == EXPECTED_LEN then
        local good, tag, session, sequence, time_us, x, y, z, qw, qx, qy, qz, marker =
          pcall(string.unpack, FORMAT, raw, offset)
        local norm = good and (qw*qw + qx*qx + qy*qy + qz*qz) or 0
        if good and tag == 'FLIO' and marker == MARKER and finite(x) and finite(y)
            and finite(z) and finite(qw) and finite(qx) and finite(qy) and finite(qz)
            and norm > 0.9 and norm < 1.1
            and (latest == nil or session ~= latest.session or sequence > latest.sequence) then
          latest = {session=session, sequence=sequence, time_us=time_us,
                    x=x, y=y, z=z, qw=qw, qx=qx, qy=qy, qz=qz,
                    received_ms=received_ms, reset=(latest == nil or session ~= latest.session)}
        end
      end
    end
  end
  LIDAR_ODOM = get_lidar_odometry()
  if (LIDAR_ODOM ~= nil) ~= was_fresh then
    was_fresh = LIDAR_ODOM ~= nil
    gcs:send_text(6, was_fresh and 'FAST-LIO: Lua pose available' or 'FAST-LIO: Lua pose stale')
  end
  if LIDAR_ODOM and (last_report == nil or (millis() - last_report):toint() >= 1000) then
    local p = LIDAR_ODOM
    local yaw = math.atan(2*(p.qw*p.qz + p.qx*p.qy), 1 - 2*(p.qy*p.qy + p.qz*p.qz))
    gcs:send_named_float('LIO_X', p.x)
    gcs:send_named_float('LIO_Y', p.y)
    gcs:send_named_float('LIO_Z', p.z)
    gcs:send_named_float('LIO_YAW', yaw)
    last_report = millis()
  end
  return update, 20
end

return update, 1000
