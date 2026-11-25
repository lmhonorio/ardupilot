--[[
Offline simulator for the Rover Lua scripts (ArduPilot scripting API mock).

  lua lua_sim.lua <scripts_dir> <trace_out> <scenario> <script1.lua> [script2.lua ...]

Runs each top-level script in its own environment, like ArduPilot, with
require() loading <scripts_dir>/modules/<name>.lua into that environment.
Scenario "can" drives the CAN path (four simulated C620 + M3508 wheels),
"pwm" the PWM path. The trace file lists GCS text, CAN current frames,
PWM overrides and saved parameters with the simulated time, so two versions
of the scripts can be compared line by line.

Needs Lua 5.3 (ArduPilot builds Lua 5.3 with 32-bit integers).
--]]

local SCRIPTS = assert(arg[1], "scripts dir")
local TRACE = assert(arg[2], "trace file")
local SCENARIO = assert(arg[3], "scenario")
local TOP = {}
for i = 4, #arg do
  TOP[#TOP + 1] = arg[i]
end

local out = assert(io.open(TRACE, "w"))
local T = 0 -- simulated time, ms
local function trace(fmt, ...)
  out:write(string.format("%8d ", T) .. string.format(fmt, ...) .. "\n")
end

---------------------------------------------------------------------------
-- uint32 time values (millis / micros)
---------------------------------------------------------------------------
local U32 = {}
U32.__index = U32
local function u32(v)
  return setmetatable({ v = v }, U32)
end
local function uv(x)
  if type(x) == "table" then
    return x.v
  end
  return x
end
function U32:toint() return math.floor(self.v) end
function U32:tofloat() return self.v + 0.0 end
U32.__sub = function(a, b) return u32(uv(a) - uv(b)) end
U32.__add = function(a, b) return u32(uv(a) + uv(b)) end
U32.__lt = function(a, b) return uv(a) < uv(b) end
U32.__le = function(a, b) return uv(a) <= uv(b) end
U32.__eq = function(a, b) return uv(a) == uv(b) end
U32.__mod = function(a, b) return u32(uv(a) % uv(b)) end
U32.__tostring = function(a) return tostring(a.v) end

---------------------------------------------------------------------------
-- Parameters
---------------------------------------------------------------------------
local PV = {}      -- name -> value
local TABLES = {}  -- key -> { prefix, size }
local function pset(name, v)
  PV[name] = v
end

local INITIAL = {
  SCR_USER1 = 0, SCR_USER2 = 0, SCR_USER3 = 0, SCR_USER4 = 0, SCR_USER5 = 2,
  SCR_VM_I_COUNT = 100000, SCR_DEBUG_OPTS = 0, SCR_HEAP_SIZE = 520000,
  RC1_TRIM = 1500, RC3_TRIM = 1500, RC3_REVERSED = 0, RC1_DZ = 30, RC3_DZ = 30,
  RCMAP_ROLL = 1, RCMAP_THROTTLE = 3,
  SERVO1_FUNCTION = 74, SERVO2_FUNCTION = 73, SERVO3_FUNCTION = 73, SERVO4_FUNCTION = 74,
  SERVO5_FUNCTION = 0, SERVO6_FUNCTION = 0, SERVO7_FUNCTION = 0, SERVO8_FUNCTION = 0,
  CAN_P1_DRIVER = 1, CAN_P2_DRIVER = 0,
  INS_GYRO_FILTER = 20, GCS_PID_MASK = 0,
  ATC_STR_RAT_FF = 0.28, ATC_STR_RAT_P = 0.14, ATC_STR_RAT_I = 0.14, ATC_STR_RAT_D = 0.002,
  ATC_STR_RAT_FLTT = 10, ATC_STR_RAT_FLTD = 10, ATC_STR_RAT_FLTE = 0,
  ATC_SPEED_FF = 0, ATC_SPEED_P = 0.68, ATC_SPEED_I = 0.68, ATC_SPEED_D = 0,
  ATC_SPEED_FLTT = 0, ATC_SPEED_FLTD = 0, ATC_SPEED_FLTE = 0,
  CRUISE_SPEED = 1.0, CRUISE_THROTTLE = 67,
  -- saved values of the script tables (applied when the script adds them)
  ESC_CAN = 1, ESC_RPM = 120, ESC_CUR = 20, ESC_IMAX = 15, ESC_P = 0.134, ESC_I = 0.150,
  ESC_LMASK = 9, ESC_ACC = 800, ESC_OPT = 1, ESC_WT_BW = 5, ESC_WT_RELAY = 25, ESC_GCS_HZ = 4,
  THRO_INV = 1, MIXER_FREQ = 50,
}
local SAVED = {}
for k, v in pairs(INITIAL) do
  if k:match("^ESC_") or k:match("^THRO_") or k:match("^MIXER_") then
    SAVED[k] = v
  else
    PV[k] = v
  end
end
if SCENARIO == "pwm" then
  SAVED.ESC_CAN = 0
end
if SCENARIO == "tc0" then
  SAVED.ESC_TC = 0
end

local param = {}
function param:add_table(key, prefix, n)
  local t = TABLES[key]
  if t ~= nil then
    return t.prefix == prefix and n <= 63
  end
  TABLES[key] = { prefix = prefix, size = n }
  return true
end
function param:add_param(key, idx, name, default)
  local t = TABLES[key]
  if t == nil or idx > t.size then
    return false
  end
  local full = t.prefix .. name
  if PV[full] == nil then
    if SAVED[full] ~= nil then
      PV[full] = SAVED[full]
    else
      PV[full] = default
    end
  end
  return true
end
function param:get(name)
  return PV[name]
end
function param:set(name, v)
  if PV[name] == nil then
    return false
  end
  PV[name] = v
  return true
end
function param:set_and_save(name, v)
  if PV[name] == nil then
    return false
  end
  PV[name] = v
  trace("SAVE %s=%.6g", name, v)
  return true
end

local ParamObj = {}
ParamObj.__index = ParamObj
local function Parameter(name)
  local o = setmetatable({}, ParamObj)
  if name ~= nil then
    o.name = name
  end
  return o
end
function ParamObj:init(name)
  if PV[name] == nil then
    return false
  end
  self.name = name
  return true
end
function ParamObj:get()
  return PV[self.name]
end
function ParamObj:set(v)
  return param:set(self.name, v)
end
function ParamObj:set_and_save(v)
  return param:set_and_save(self.name, v)
end

---------------------------------------------------------------------------
-- Vehicle state and plant
---------------------------------------------------------------------------
local V = {
  armed = false, mode = 0, yaw = 0, yaw_rate = 0, speed = 0,
  rc = { [1] = 1500, [3] = 1500 },
  thr_out = 0, str_out = 0, -- ArduPilot controller outputs (-1..1)
  nav_idx = 0, mission_state = 1, items = {},
  servo = {}, servo_until = {},
  auth = nil,
}

-- Wheels: current -> wheel rpm, Kv rpm/s per A, Coulomb friction, viscous loss
local W = {}
for i = 1, 4 do
  W[i] = { rpm = 0, cur = 0, cmd = 0, temp = 30 }
end
local KV = { 30, 34, 28, 32 }
local FRIC = { 2.0, 2.0, 2.0, 2.0 }
-- Rover hung on a diagonal: wheels 1 and 3 hold it (19 A to break free),
-- 2 and 4 hang in the air (light and free)
if SCENARIO == "tc" or SCENARIO == "tc0" then
  FRIC = { 19.0, 0.3, 19.0, 0.3 }
  KV = { 30, 150, 28, 150 }
end
local VISC = 0.02
local GEAR = 19.2032
local LEFT = { [1] = true, [4] = true } -- ESC_LMASK = 9

local can_rx = {} -- frames waiting for the script
local CAN_BUF = 25

local Frame = {}
Frame.__index = Frame
local function CANFrame()
  return setmetatable({ _id = 0, _dlc = 0, _d = { 0, 0, 0, 0, 0, 0, 0, 0 } }, Frame)
end
function Frame:id(v) if v ~= nil then self._id = v end return self._id end
function Frame:id_signed() return self._id end
function Frame:dlc(v) if v ~= nil then self._dlc = v end return self._dlc end
function Frame:data(i, v)
  if v ~= nil then
    self._d[i + 1] = v
  end
  return self._d[i + 1]
end
function Frame:isExtended() return false end
function Frame:isRemoteTransmissionRequest() return false end
function Frame:isErrorFrame() return false end

local function i16(v)
  v = math.floor(v)
  if v < 0 then v = v + 65536 end
  return (v >> 8) & 0xFF, v & 0xFF
end
local function from_i16(hi, lo)
  local v = ((hi & 0xFF) << 8) | (lo & 0xFF)
  if v >= 32768 then v = v - 65536 end
  return v
end

local can_active_cmd_ms = -1000
local driver = {}
function driver:add_filter() return true end
function driver:read_frame()
  return table.remove(can_rx, 1)
end
function driver:write_frame(f, _timeout)
  if f._id == 0x200 then
    local s = {}
    for i = 1, 4 do
      local raw = from_i16(f._d[2 * i - 1], f._d[2 * i])
      W[i].cmd = raw / (16384 / 20)
      s[#s + 1] = tostring(raw)
    end
    can_active_cmd_ms = T
    trace("CAN %s", table.concat(s, " "))
  end
  return true
end

local function plant_step(dt)
  -- PWM path: C620 follows its servo pulse while no CAN command is fresh
  local can_live = (T - can_active_cmd_ms) < 50
  for i = 1, 4 do
    local w = W[i]
    local target
    if can_live then
      target = w.cmd
    else
      local ch = i - 1
      local pwm = (V.servo_until[ch] or -1) >= T and V.servo[ch] or 0
      if pwm == 0 or pwm == nil then
        target = 0
      else
        target = (pwm - 1500) / 500 * 20
      end
    end
    if not V.armed then
      target = can_live and target or 0
    end
    w.cur = w.cur + (target - w.cur) * math.min(1, dt / 0.004)
    local fric = 0
    local fa = FRIC[i]
    if math.abs(w.rpm) > 0.5 then
      fric = (w.rpm > 0) and fa or -fa
    elseif math.abs(w.cur) <= fa then
      fric = w.cur
    else
      fric = (w.cur > 0) and fa or -fa
    end
    w.rpm = w.rpm + KV[i] * (w.cur - fric - VISC * w.rpm) * dt
  end
  -- Body: forward is negative wheel rpm on this rover
  local l = (W[1].rpm + W[4].rpm) / 2
  local r = (W[2].rpm + W[3].rpm) / 2
  V.speed = -(l + r) / 2 * 0.0125 -- m/s
  V.yaw_rate = (r - l) * 0.012     -- rad/s
  V.yaw = V.yaw + V.yaw_rate * dt
  while V.yaw > math.pi do V.yaw = V.yaw - 2 * math.pi end
  while V.yaw < -math.pi do V.yaw = V.yaw + 2 * math.pi end
  -- C620 feedback, 1 kHz each; buffer drops the newest when full
  for i = 1, 4 do
    local f = CANFrame()
    f._id = 0x200 + i
    f._dlc = 8
    local rotor = math.floor(W[i].rpm * GEAR + 0.5)
    local h, lo = i16(rotor)
    f._d[3], f._d[4] = h, lo
    h, lo = i16(W[i].cur * (16384 / 20))
    f._d[5], f._d[6] = h, lo
    f._d[7] = W[i].temp
    if #can_rx < CAN_BUF then
      can_rx[#can_rx + 1] = f
    end
  end
end

---------------------------------------------------------------------------
-- ArduPilot bindings
---------------------------------------------------------------------------
local Vec = {}
Vec.__index = Vec
local function vec(x, y, z) return setmetatable({ _x = x, _y = y, _z = z }, Vec) end
function Vec:x() return self._x end
function Vec:y() return self._y end
function Vec:z() return self._z end

local api = {}
api.param = param
api.Parameter = Parameter
api.millis = function() return u32(T) end
api.micros = function() return u32(T * 1000) end
api.gcs = {
  send_text = function(_, sev, text) trace("GCS %d %s", sev, text) end,
  send_named_float = function(_, _name, _v) end,
}
api.arming = {
  is_armed = function() return V.armed end,
  get_aux_auth_id = function() return 1 end,
  set_aux_auth_passed = function() V.auth = nil end,
  set_aux_auth_failed = function(_, _id, msg)
    if V.auth ~= msg then trace("AUTH %s", msg) end
    V.auth = msg
  end,
}
api.vehicle = {
  get_mode = function() return V.mode end,
  set_mode = function(_, m)
    trace("MODE %d", m)
    V.mode = m
    return true
  end,
  get_control_output = function(_, ch)
    if ch == 3 then return V.thr_out end
    if ch == 4 then return V.str_out end
    return 0
  end,
  get_steering_and_throttle = function() return V.str_out, V.thr_out end,
  get_wp_crosstrack_error_m = function() return 0.1 end,
}
api.SRV_Channels = {
  set_output_pwm_chan_timeout = function(_, ch, pwm, timeout)
    if V.servo[ch] ~= pwm then
      trace("PWM %d %d", ch, pwm)
    end
    V.servo[ch] = pwm
    V.servo_until[ch] = T + timeout
  end,
  get_output_scaled = function() return 0 end,
  get_safety_state = function() return false end,
  get_emergency_stop = function() return false end,
}
local RcChan = {}
RcChan.__index = RcChan
function RcChan:norm_input_dz()
  local p = V.rc[self.ch] or 1500
  if math.abs(p - 1500) <= 30 then return 0 end
  return (p - 1500) / 500
end
api.rc = {
  get_pwm = function(_, ch) return V.rc[ch] or 1500 end,
  get_channel = function(_, ch) return setmetatable({ ch = ch }, RcChan) end,
  get_aux_cached = function() return 0 end,
  has_valid_input = function() return true end,
}
api.ahrs = {
  get_yaw = function() return V.yaw end,
  get_pitch = function() return 0 end,
  get_roll = function() return 0 end,
  get_gyro = function() return vec(0, 0, V.yaw_rate) end,
  get_velocity_NED = function() return vec(V.speed * math.cos(V.yaw), V.speed * math.sin(V.yaw), 0) end,
  earth_to_body = function(_, v)
    local c, s = math.cos(V.yaw), math.sin(V.yaw)
    return vec(c * v:x() + s * v:y(), -s * v:x() + c * v:y(), 0)
  end,
  healthy = function() return true end,
  get_relative_position_NED_origin = function() return vec(0, 0, 0) end,
}
local Item = {}
Item.__index = Item
function Item:command() return self.cmd end
function Item:z() return self.zv end
api.mission = {
  get_current_nav_index = function() return V.nav_idx end,
  get_item = function(_, idx)
    local it = V.items[idx]
    if it == nil then return nil end
    return setmetatable({ cmd = it[1], zv = it[2] }, Item)
  end,
  state = function() return V.mission_state end,
}
api.CAN = { get_device = function(_, _n) return driver end }
api.CANFrame = CANFrame
api.logger = { write = function() end }
api.esc_telem = { update_telem_data = function() end, update_rpm = function() end }
local Telem = {}
Telem.__index = Telem
function Telem:motor_temp_cdeg() end
function Telem:current() end
api.ESCTelemetryData = function() return setmetatable({}, Telem) end
api.AR_AttitudeControl = { get_srate = function() return 0, 0 end }
api.battery = { has_failsafed = function() return false end }

---------------------------------------------------------------------------
-- Script loading, ArduPilot style
---------------------------------------------------------------------------
local function loadfile_env(path, env)
  local f = io.open(path, "rb")
  if not f then
    return nil, "cannot open " .. path
  end
  local src = f:read("a")
  f:close()
  return load(src, "@" .. path, "t", env)
end

local function new_env()
  local env = {}
  for k, v in pairs(_G) do
    if k ~= "arg" and k ~= "os" and k ~= "collectgarbage" and k ~= "dofile" and k ~= "debug" and k ~= "mem_kb"
        and k ~= "loadfile" and k ~= "require" and k ~= "package" then
      env[k] = v
    end
  end
  for k, v in pairs(api) do
    env[k] = v
  end
  env._G = env
  env._LOADED = {}
  env.package = { path = "./?.lua", loaded = env._LOADED }
  env.require = function(name)
    -- ArduPilot (lua/src/loadlib.c ll_require) takes the environment from the
    -- FIRST upvalue of the function the scheduler is running, not from the
    -- caller. Reproduce that, so a script whose running function does not
    -- have _ENV first fails here as it does on the board.
    local cur = CURRENT_FN
    if cur ~= nil and debug ~= nil then
      local uname, uval = debug.getupvalue(cur, 1)
      if type(uval) ~= "table" then
        error("attempt to index a " .. type(uval) .. " value (ArduPilot require: first upvalue of the running function is '" .. tostring(uname) .. "')", 2)
      end
    end
    if env._LOADED[name] ~= nil then
      return env._LOADED[name]
    end
    local path = SCRIPTS .. "/modules/" .. name .. ".lua"
    local chunk, err = loadfile_env(path, env)
    if not chunk then
      error("module '" .. name .. "' not found: " .. tostring(err), 2)
    end
    local r = chunk(name, path)
    if r == nil then r = true end
    env._LOADED[name] = r
    return r
  end
  return env
end

CURRENT_FN = nil
local tasks = {}
for _, file in ipairs(TOP) do
  local env = new_env()
  local chunk = assert(loadfile_env(SCRIPTS .. "/" .. file, env))
  tasks[#tasks + 1] = { name = file, fn = chunk, due = 0, first = true }
end

local function run_task(t)
  CURRENT_FN = t.fn
  local ok, fn, delay
  if t.first then
    t.first = false
    ok, fn, delay = pcall(t.fn)
  else
    ok, fn, delay = pcall(t.fn)
  end
  if not ok then
    trace("SCRIPT_ERROR %s %s", t.name, tostring(fn))
    t.dead = true
    return
  end
  if fn == nil then
    trace("SCRIPT_END %s", t.name)
    t.dead = true
    return
  end
  t.fn = fn
  t.due = T + math.max(1, math.floor(delay or 1))
  -- ArduPilot's runtime line while SCR_DEBUG_OPTS bit 1 is set (memory not modelled)
  if ((PV.SCR_DEBUG_OPTS or 0) & 2) ~= 0 then
    trace("GCS 7 Lua: Time: 0 Mem: 0 + 0")
  end
end

---------------------------------------------------------------------------
-- Scenario
---------------------------------------------------------------------------
local EVENTS = {}
local function at(ms, f) EVENTS[#EVENTS + 1] = { ms, f } end

local function auto_outputs()
  -- a smooth controller output while driving
  local s = T / 1000
  V.thr_out = 0.45 + 0.05 * math.sin(s)
  V.str_out = 0.15 * math.sin(0.7 * s)
end

local MODE = { MANUAL = 0, ACRO = 1, STEERING = 3, HOLD = 4, CIRCLE = 9, AUTO = 10 }
local scenario_end

local function main_scenario()
  at(6000, function() V.armed = true end)
  at(7000, function() V.rc[3] = 1250 V.rc[1] = 1580 end)    -- forward + right
  at(9000, function() V.rc[3] = 1700 V.rc[1] = 1500 end)    -- backward
  at(10500, function() V.rc[3] = 1500 V.rc[1] = 1500 end)   -- centred
  -- AUTO with a pass-through waypoint, then a yaw alignment to 90 deg
  at(12000, function()
    V.items = { [1] = { 16, -1 }, [2] = { 16, 90 }, [3] = { 16, -1 } }
    V.nav_idx = 1
    V.mode = MODE.AUTO
  end)
  at(15000, function() V.nav_idx = 2 end)
  at(19000, function() V.nav_idx = 3 end)
  -- wheel step tune (CAN only): chosen while disarmed, as on the rover (the
  -- tune module loads only while disarmed), then armed in HOLD
  at(44000, function()
    V.armed = false
    V.mode = MODE.HOLD
    V.thr_out, V.str_out = 0, 0
    if SCENARIO == "can" then PV.SCR_USER1 = 4 end
    if SCENARIO == "can5" then PV.SCR_USER1 = 5 end
  end)
  at(45000, function() V.armed = true end)
  -- yaw relay tune in HOLD
  at(80000, function() PV.SCR_USER1 = 0 end)
  at(81000, function() PV.SCR_USER1 = 2 end)
  -- QuickTune in Circle: selected while disarmed, then armed
  at(198000, function()
    V.armed = false
    PV.SCR_USER1 = 1
  end)
  at(200000, function()
    V.armed = true
    V.mode = MODE.CIRCLE
  end)
  -- Circle relay tune
  at(260000, function() PV.SCR_USER1 = 3 end)
  at(300000, function()
    PV.SCR_USER1 = 0
    V.mode = MODE.MANUAL
  end)
  at(302000, function() V.armed = false end)
  scenario_end = 306000
end

-- Rover hung on a diagonal (see FRIC / KV): slow forward stick for 10 s.
-- The trace gets wheel rpm and current every 500 ms.
local function traction_scenario()
  at(6000, function() V.armed = true end)
  at(7000, function() V.rc[3] = 1382 end) -- slow forward, about 25 rpm target
  at(17000, function() V.rc[3] = 1500 end)
  at(19000, function() V.armed = false end)
  scenario_end = 20000
end

local TRACTION = SCENARIO == "tc" or SCENARIO == "tc0"
if TRACTION then
  traction_scenario()
else
  main_scenario()
end

local ei = 1
local PLANT_DT = 0.001
local mem_done = false
while T <= scenario_end do
  while ei <= #EVENTS and EVENTS[ei][1] <= T do
    EVENTS[ei][2]()
    ei = ei + 1
  end
  if V.mode == MODE.AUTO or V.mode == MODE.CIRCLE then
    auto_outputs()
  end
  plant_step(PLANT_DT)
  if TRACTION and T % 500 == 0 then
    trace("W rpm %6.1f %6.1f %6.1f %6.1f  A %5.1f %5.1f %5.1f %5.1f", W[1].rpm, W[2].rpm, W[3].rpm, W[4].rpm,
      W[1].cur, W[2].cur, W[3].cur, W[4].cur)
  end
  for _, t in ipairs(tasks) do
    if not t.dead and T >= t.due then
      run_task(t)
    end
  end
  if not mem_done and T == 5000 then
    mem_done = true
    if mem_kb ~= nil then -- host interpreter built with a byte counter
      io.stderr:write(string.format("Lua memory after load: %.0f kB\n", mem_kb()))
    end
  end
  T = T + 1
end
trace("END")
out:close()
