-- ESC CAN driver for DJI C620 / M3508 on stock ArduPilot Rover.
-- Copy to APM/scripts on the SD card together with rover-controlallocation.lua
-- and modules/autotune.lua. No custom firmware is required.
--
-- CAN_Px_DRIVER = 1 or 2, CAN_Px_BITRATE = 1000000, CAN_Dx_PROTOCOL = 10 (Scripting)
--
-- Lua scripts do not share globals. rover-controlallocation.lua sends the
-- left/right command through the RAM-only parameters ESC_CMD_L, ESC_CMD_R and
-- ESC_CMD_SEQ, and reads the driver state from ESC_STAT. Never save them.
--
-- ESC_CAN      0 = PWM (driver silent), 1 = CAN. Change only while disarmed.
-- ESC_MODE     0 = side speed PI, 1 = current follows the command
-- ESC_RPM      wheel rpm at command 1.0
-- ESC_GEAR     rotor-to-wheel ratio (M3508 P19 = 19.2032)
-- ESC_CUR      current limit, ampere
-- ESC_P _I _D _FF _IMAX   side speed PI (current in A, error in wheel rpm)
-- ESC_REV      bitmask, bit 0 = ESC1 ... reverses that motor
-- ESC_OPT      bit 0 log DJIC, bit 1 hold zero rpm while disarmed,
--              bit 2 desaturation (keep the left/right ratio when a side saturates)
-- ESC_BRK      0 = never push current against the spin, 1 = brake up to ESC_BRKCUR
-- ESC_ACC      reference acceleration limit, wheel rpm/s, both ways (0 = none = fully reactive).
--              Keep at 0 for stick/mission command with no artificial ramp.
-- ESC_BRKCUR   braking current limit, % of ESC_CUR
-- ESC_LOC_P    correction between the two wheels of one side, A per rpm (0 = off)
-- ESC_LOC_I    integral of that correction, A per rpm·s (0 = off). Gets the wheel that
--              is stuck on static friction moving at very low speed.
-- ESC_LOC_MAX  limit of that correction, % of ESC_CUR
-- ESC_SLIP     wheel-to-side rpm difference reported as slip (0 = off). Also used as the
--              divergence threshold for traction split (default 40 rpm if 0). When a side
--              diverges, side PI tracks the lagging (ground) wheel, the spinning wheel is
--              cut to brake-only, and the stuck wheel gets the full side current.
-- ESC_LMASK    bitmask of the ESCs (1..4) on the left side; the others are right
--              (ESC_LMASK=9 → ESC1+ESC4 left, ESC2+ESC3 right)
-- ESC_WT_RELAY wheel autotune relay amplitude, % of ESC_CUR
-- ESC_WT_RPM   wheel autotune operating point, % of ESC_RPM
--
-- ESC_FR1..4   breakaway current of each wheel, A (0 = unknown). Wheels of one side
--              get the difference to their side mean as friction compensation.
--
-- Wheel autotune: SCR_USER1 = 4, armed in HOLD, ESC_CAN = 1. Saves ESC_FF, ESC_P,
-- ESC_I and sets SCR_USER1 back to 0. The rover drives about 1-2 m forward
-- and comes back in reverse.
-- Breakaway test: SCR_USER1 = 5, WHEELS IN THE AIR, armed in HOLD. Current rises on
-- every wheel until it turns, forward then reverse. Saves ESC_FR1..4 and ESC_LOC_MAX.
-- Local autotune: SCR_USER1 = 6, on the ground, armed in HOLD. Relay on the speed
-- difference between the two wheels of each side. Saves ESC_LOC_P and ESC_LOC_I.

package.path = package.path .. ';/scripts/modules/?.lua'
local Autotune = require("autotune")

local K = {
  NUM_ESC = 4,
  MAX_FB = 8,
  CMD_ID = 0x200,
  FB_BASE = 0x200,
  COUNTS_PER_REV = 8192,
  RAW_PER_AMP = 16384 / 20,
  RAW_MAX = 16384,
  FEEDBACK_TIMEOUT_MS = 100,
  LOOP_MS = 10,
  MAX_FRAMES = 12,
  CMD_STALE_MS = 500,
  SAT_HOLD_MS = 100,
  SLIP_MS = 100,
  MOVING_RPM = 5,
  HOLD_MODE = 4,
  WHEEL_TUNE_SELECTOR = 4,
  BRK_SELECTOR = 5,
  LOC_SELECTOR = 6,
  BRK_RAMP = 0.5,
  BRK_MAX_FRAC = 0.6,
  BRK_RPM = 2,
  BRK_CONFIRM_S = 0.05,
  BRK_REST_S = 2,
  TUNE_SETTLE_S = 1.0,
  TUNE_AVERAGE_S = 1.0,
  TUNE_STICK_US = 100,
  STAT_CAN = 1,
  STAT_SAT = 2,
  STAT_TUNE_RUN = 4,
  STAT_TUNE_OK = 8,
  STAT_TUNE_FAIL = 16,
  STAT_SLIP = 32,
  PARAM_KEY = 74,
  PREFIX = "ESC_",
  TELEM_CURRENT = 1 << 3,
  TELEM_MOTOR_TEMP = 1 << 1,
}

local PARAM_SPEC = {
  { 1, "CAN", 0 },
  { 2, "MODE", 0 },
  { 3, "RPM", 400 },
  { 4, "GEAR", 19.2032 },
  { 5, "CUR", 10 },
  { 6, "P", 0.03 },
  { 7, "I", 0.3 },
  { 8, "D", 0 },
  { 9, "FF", 0 },
  { 10, "IMAX", 10 },
  { 11, "REV", 0 },
  { 12, "OPT", 1 },
  { 13, "BRK", 1 },
  { 14, "ACC", 0 },
  { 15, "BRKCUR", 30 },
  { 16, "LOC_P", 0 },
  { 17, "LOC_MAX", 15 },
  { 18, "SLIP", 60 },
  { 19, "LMASK", 6 },
  { 20, "WT_RELAY", 25 },
  { 21, "WT_RPM", 15 },
  { 22, "CMD_L", 0 },
  { 23, "CMD_R", 0 },
  { 24, "CMD_SEQ", 0 },
  { 25, "STAT", 0 },
  { 26, "LOC_I", 0.5 },
  { 27, "FR1", 0 },
  { 28, "FR2", 0 },
  { 29, "FR3", 0 },
  { 30, "FR4", 0 },
}

local MAV_SEVERITY = { CRITICAL = 2, ERROR = 3, WARNING = 4, NOTICE = 5, INFO = 6 }

local P = {}
local params_ready = false
local scr_user1 = Parameter()
local scr_user5 = Parameter()
local rc1_trim = Parameter()
local rc3_trim = Parameter()
local user_params_bound = false

local driver = nil
-- Stay in PWM until ESC_CAN is read. Starting as CAN would zero MAIN 1..4.
local latched_can = false
local auth_id = nil
local last_switch_warn = 0
local last_lost_warn = 0
local last_driver_warn = 0
local last_mask_warn = 0
local diag = {n = {0, 0, 0, 0, 0, 0, 0, 0}, other = 0, other_id = 0, last_ms = 0, last_loop_ms = 0, max_dt = 0}
local last_slip_warn = 0
local last_tune_warn = 0
local last_log = 0
local last_telem = 0
local last_us = nil
local lost_mask = 0
local announced_driver = false
local telem_slots = K.NUM_ESC
local telem_warned = false
local published_stat = -1
local sat_ms = nil
local slip_active = false

local esc = {}
local slip_since = {}
local function reset_esc(i)
  esc[i] = {
    angle = 0,
    have_angle = false,
    rotor_counts = 0,
    rotor_rpm = 0,
    current_raw = 0,
    temperature = 0,
    last_ms = nil,
    detected = false,
    wheel_rpm = 0,
    current_a = 0,
    loc_int = 0,
  }
  slip_since[i] = nil
end
for i = 1, K.MAX_FB do
  reset_esc(i)
end

-- Side 1 = left, side 2 = right
local side = {}
local function reset_side(s)
  side[s] = {
    integrator = 0,
    last_error = 0,
    rpm = 0,
    rpm_mean = 0,
    rpm_lag = 0,
    diverge_rpm = 40,
    diverged = false,
    ref_rpm = 0,
    target_rpm = 0,
    current = 0,
    saturated = false,
    goal = 0,
    interp_value = 0,
  }
end
reset_side(1)
reset_side(2)

local cmd_seq_seen = nil
local cmd_seq_ms = nil

local tune = { state = "idle" }

-- Rover 4.7 gives millis() and micros() as uint32_t. tonumber() of that value is nil.
local function as_number(value)
  if type(value) == "number" then
    return value
  end
  if value ~= nil and value.tofloat ~= nil then
    return value:tofloat()
  end
  return tonumber(value)
end

local function now_ms()
  return as_number(millis())
end

local function constrain(value, lo, hi)
  if value > hi then
    return hi
  elseif value < lo then
    return lo
  end
  return value
end

local function finite(value)
  return type(value) == "number" and value == value and math.abs(value) < math.huge
end

local function g(name)
  local value = P[name]:get()
  if not finite(value) then
    return 0
  end
  return value
end

local function set_param(name, value)
  P[name]:set(value)
end

-- EEPROM / servo maps: keep helpers as table methods so they do not burn locals
local M = {
  save_q = {},
  dir = { 1, 1, 1, 1, 1, 1, 1, 1 },
  side = { 2, 2, 2, 2, 2, 2, 2, 2 },
  thr = { false, false, false, false, false, false, false, false },
  map_ms = 0,
}

function M.queue_save(obj, value)
  obj:set(value)
  M.save_q[#M.save_q + 1] = { obj = obj, value = value }
end

function M.drain_saves()
  if #M.save_q == 0 then
    return
  end
  local item = table.remove(M.save_q, 1)
  item.obj:set_and_save(item.value)
end

function M.refresh_maps(force)
  local now = now_ms()
  if not force and now - M.map_ms < 1000 then
    return
  end
  M.map_ms = now
  local rev = math.floor(g("REV"))
  local lmask = math.floor(g("LMASK"))
  for i = 0, K.MAX_FB - 1 do
    M.dir[i + 1] = ((rev & (1 << i)) ~= 0) and -1 or 1
    M.side[i + 1] = ((lmask & (1 << i)) ~= 0) and 1 or 2
    local value = param:get(string.format("SERVO%d_FUNCTION", i + 1)) or 0
    local fn = math.floor(value)
    M.thr[i + 1] = (fn == 70 or fn == 73 or fn == 74)
  end
end

local function ensure_params()
  if params_ready then
    return true
  end
  -- add_table returns true again after a script restart because the table already exists
  if not param:add_table(K.PARAM_KEY, K.PREFIX, #PARAM_SPEC) then
    return false
  end
  for n = 1, #PARAM_SPEC do
    local item = PARAM_SPEC[n]
    if not param:add_param(K.PARAM_KEY, item[1], item[2], item[3]) then
      gcs:send_text(MAV_SEVERITY.WARNING, string.format("ESC: failed to add %s%s", K.PREFIX, item[2]))
      return false
    end
    local obj = Parameter()
    if not obj:init(K.PREFIX .. item[2]) then
      return false
    end
    P[item[2]] = obj
  end
  if not user_params_bound then
    user_params_bound = scr_user1:init("SCR_USER1") and scr_user5:init("SCR_USER5") and
      rc1_trim:init("RC1_TRIM") and rc3_trim:init("RC3_TRIM")
  end
  -- Commands from a previous boot must never look fresh.
  set_param("STAT", 0)
  params_ready = true
  gcs:send_text(MAV_SEVERITY.INFO, "ESC: ESC_* parameters ready")
  local vm = param:get("SCR_VM_I_COUNT") or 0
  if vm < 50000 then
    gcs:send_text(MAV_SEVERITY.WARNING, string.format("ESC: set SCR_VM_I_COUNT=100000 (now %.0f) and reboot", vm))
  end
  return true
end

local function direction(idx)
  return M.dir[idx + 1]
end

local function side_of(idx)
  return M.side[idx + 1]
end

local function mask_valid()
  local lmask = math.floor(g("LMASK")) & 0xFF
  return lmask ~= 0 and lmask ~= 0xFF
end

local function is_throttle(idx)
  return M.thr[idx + 1]
end

local function be_i16(hi, lo)
  local value = ((hi & 0xFF) << 8) | (lo & 0xFF)
  if value >= 32768 then
    value = value - 65536
  end
  return value
end

local function script_mixer_active()
  return user_params_bound and scr_user5:get() == 2
end

local function selector()
  if not user_params_bound then
    return 0
  end
  return scr_user1:get() or 0
end

local function healthy(idx)
  local fb = esc[idx + 1]
  if not fb.detected or fb.last_ms == nil then
    return false
  end
  return now_ms() - fb.last_ms < K.FEEDBACK_TIMEOUT_MS
end

local function all_healthy()
  local saw = false
  for i = 0, K.MAX_FB - 1 do
    if is_throttle(i) or esc[i + 1].detected then
      saw = true
      if not healthy(i) then
        return false
      end
    end
  end
  return saw
end

local function kill_pwm()
  -- MAIN OUT 1..4 are wired to the C620 PWM inputs. Any pulse, even trim
  -- at 1500, knocks the ESC out of CAN. Keep width 0 with a long override
  -- so Rover cannot briefly write trim between script cycles.
  SRV_Channels:set_output_pwm_chan_timeout(0, 0, 3000)
  SRV_Channels:set_output_pwm_chan_timeout(1, 0, 3000)
  SRV_Channels:set_output_pwm_chan_timeout(2, 0, 3000)
  SRV_Channels:set_output_pwm_chan_timeout(3, 0, 3000)
end

local function reset_command()
  cmd_seq_seen = nil
  cmd_seq_ms = nil
end

local function reset_control()
  reset_side(1)
  reset_side(2)
  for i = 1, K.MAX_FB do
    esc[i].current_a = 0
    esc[i].loc_int = 0
  end
end

local function tune_message(severity, text)
  gcs:send_text(severity, "WheelTune: " .. text)
end

local function tune_reset()
  tune = { state = "idle" }
end

local function apply_switch()
  local want_can = g("CAN") >= 0.5
  if want_can == latched_can then
    return
  end
  if arming:is_armed() then
    local now = now_ms()
    if now - last_switch_warn >= 5000 then
      last_switch_warn = now
      gcs:send_text(MAV_SEVERITY.WARNING, "ESC: disarm to switch ESC_CAN")
    end
    return
  end
  latched_can = want_can
  for i = 1, K.NUM_ESC do
    reset_esc(i)
  end
  reset_control()
  reset_command()
  tune_reset()
  last_us = nil
  lost_mask = 0
  sat_ms = nil
  if latched_can then
    -- Kill PWM first. The C620 must not see a servo pulse when CAN starts.
    kill_pwm()
    -- Drop stale RX from the PWM to CAN transition before accepting feedback.
    for _ = 1, K.MAX_FRAMES do
      if not driver:read_frame() then
        break
      end
    end
    -- Fully reactive: no quiet hold / stick-center gate after CAN latch.
    gcs:send_text(MAV_SEVERITY.INFO, "ESC: CAN ON (reactive), PWM zero")
  else
    gcs:send_text(MAV_SEVERITY.INFO, "ESC: CAN OFF (PWM)")
  end
end

local function accept_frame(frame)
  if frame:isExtended() or frame:isRemoteTransmissionRequest() or frame:isErrorFrame() or frame:dlc() ~= 8 then
    return
  end
  local id = as_number(frame:id())
  if id == nil then
    return
  end
  id = math.floor(id) & 0x7FF
  if id <= K.FB_BASE or id > K.FB_BASE + K.MAX_FB then
    diag.other = diag.other + 1
    diag.other_id = id
    return
  end
  diag.n[id - K.FB_BASE] = diag.n[id - K.FB_BASE] + 1
  local idx = id - K.FB_BASE - 1
  local fb = esc[idx + 1]
  local angle = (((frame:data(0) & 0xFF) << 8) | (frame:data(1) & 0xFF)) % K.COUNTS_PER_REV
  if fb.have_angle then
    local delta = angle - fb.angle
    if delta > K.COUNTS_PER_REV / 2 then
      delta = delta - K.COUNTS_PER_REV
    elseif delta < -K.COUNTS_PER_REV / 2 then
      delta = delta + K.COUNTS_PER_REV
    end
    fb.rotor_counts = fb.rotor_counts + delta
  end
  fb.angle = angle
  fb.have_angle = true
  fb.rotor_rpm = be_i16(frame:data(2), frame:data(3))
  fb.current_raw = be_i16(frame:data(4), frame:data(5))
  fb.temperature = frame:data(6) & 0xFF
  fb.last_ms = now_ms()
  if not fb.detected then
    fb.detected = true
    gcs:send_text(MAV_SEVERITY.INFO, string.format("ESC: found ESC %d (%s)", idx + 1,
      side_of(idx) == 1 and "left" or "right"))
  end
end

local function drain()
  -- Four C620s send about 4000 frames/s. Reading until the buffer stays
  -- empty never returns and the scripting VM reports exceeded time.
  for _ = 1, K.MAX_FRAMES do
    local frame = driver:read_frame()
    if not frame then
      return
    end
    accept_frame(frame)
  end
end

local function put_i16(frame, index, value)
  value = math.floor(constrain(value, -K.RAW_MAX, K.RAW_MAX))
  if value < 0 then
    value = value + 65536
  end
  frame:data(index, (value >> 8) & 0xFF)
  frame:data(index + 1, value & 0xFF)
end

local function send_group(std_id, raw, first)
  local frame = CANFrame()
  frame:id(std_id)
  frame:dlc(8)
  for i = 0, 3 do
    put_i16(frame, i * 2, raw[first + i])
  end
  driver:write_frame(frame, 1000)
end

-- Rover writes ThrottleLeft/Right as -1000..1000 and Throttle as -100..100.
local function servo_side_norms()
  local left = SRV_Channels:get_output_scaled(73) or 0
  local right = SRV_Channels:get_output_scaled(74) or 0
  if left == 0 and right == 0 then
    local throttle = (SRV_Channels:get_output_scaled(70) or 0) / 100
    return constrain(throttle, -1, 1), constrain(throttle, -1, 1)
  end
  return constrain(left / 1000, -1, 1), constrain(right / 1000, -1, 1)
end

--[[
Update the commanded side norms. Reactive path: each new ESC_CMD_SEQ applies
L/R immediately (no inter-sample interpolation).
--]]
local function update_command(now, dt)
  if not script_mixer_active() then
    local left, right = servo_side_norms()
    side[1].goal, side[2].goal = left, right
    side[1].interp_value, side[2].interp_value = left, right
    return
  end
  local seq = g("CMD_SEQ")
  if cmd_seq_seen == nil then
    -- First value seen after boot or a switch is not a fresh command.
    cmd_seq_seen = seq
  elseif seq ~= cmd_seq_seen then
    cmd_seq_seen = seq
    cmd_seq_ms = now
    side[1].goal = constrain(g("CMD_L"), -1, 1)
    side[2].goal = constrain(g("CMD_R"), -1, 1)
    side[1].interp_value = side[1].goal
    side[2].interp_value = side[2].goal
  end
  if cmd_seq_ms == nil or now - cmd_seq_ms > K.CMD_STALE_MS then
    for s = 1, 2 do
      side[s].goal = 0
      side[s].interp_value = 0
    end
  end
end

local function profile_reference(s, target_rpm, dt)
  local sd = side[s]
  local acc = g("ACC")
  if acc > 0 then
    local step = acc * dt
    sd.ref_rpm = sd.ref_rpm + constrain(target_rpm - sd.ref_rpm, -step, step)
  else
    sd.ref_rpm = target_rpm
  end
end

-- Current window allowed for a wheel or side turning at rpm.
local function current_window(rpm, k)
  local lo, hi = -k.max_current, k.max_current
  local brake = k.brake and k.brake_current or 0
  if rpm > K.MOVING_RPM then
    lo = -brake
  elseif rpm < -K.MOVING_RPM then
    hi = brake
  end
  return lo, hi
end

local function side_pi(s, target, dt, k)
  local sd = side[s]
  sd.target_rpm = target
  local err = target - sd.rpm
  local derivative = 0
  if dt > 0 then
    derivative = k.kd * (err - sd.last_error) / dt
  end
  sd.last_error = err
  local lo, hi = current_window(sd.rpm, k)
  local ff = k.kff * target
  local unsat = ff + k.kp * err + sd.integrator + derivative
  local saturated = (unsat >= hi and err > 0) or (unsat <= lo and err < 0)
  if not saturated then
    sd.integrator = constrain(sd.integrator + k.ki * err * dt, -k.imax, k.imax)
  end
  sd.saturated = (unsat > k.max_current and err > 0) or (unsat < -k.max_current and err < 0)
  return constrain(ff + k.kp * err + sd.integrator + derivative, lo, hi)
end

local function read_gains()
  local max_current = constrain(g("CUR"), 0, 20)
  return {
    gear = math.max(g("GEAR"), 1),
    max_rpm = math.max(g("RPM"), 1),
    max_current = max_current,
    imax = constrain(g("IMAX"), 0, max_current),
    kp = g("P"),
    ki = g("I"),
    kd = g("D"),
    kff = g("FF"),
    speed_mode = g("MODE") < 0.5,
    opt = math.floor(g("OPT")),
    brake = g("BRK") >= 0.5,
    brake_current = max_current * constrain(g("BRKCUR"), 0, 100) / 100,
    loc_p = math.max(g("LOC_P"), 0),
    loc_i = math.max(g("LOC_I"), 0),
    loc_max = max_current * constrain(g("LOC_MAX"), 0, 100) / 100,
    fr = { g("FR1"), g("FR2"), g("FR3"), g("FR4") },
    slip = math.max(g("SLIP"), 0),
  }
end

local function rc_centered()
  local t1 = rc1_trim:get() or 1500
  local t3 = rc3_trim:get() or 1500
  local p1 = rc:get_pwm(1) or t1
  local p3 = rc:get_pwm(3) or t3
  return math.abs(p1 - t1) < K.TUNE_STICK_US and math.abs(p3 - t3) < K.TUNE_STICK_US
end

local function tune_wanted(armed)
  return selector() == K.WHEEL_TUNE_SELECTOR and armed and vehicle:get_mode() == K.HOLD_MODE
end

local function tune_fail(msg)
  tune.state = "fail"
  tune_message(MAV_SEVERITY.ERROR, "failed - " .. msg .. "; gains kept")
end

-- modules/autotune.lua names its limits *_rad after the yaw use; here they are in rpm.
local function make_relay(k, hyst)
  local relay = constrain(k.max_current * constrain(g("WT_RELAY"), 1, 80) / 100, 0.5, 5)
  return Autotune:new({
    relay = relay,
    hyst_rad = hyst or math.max(k.max_rpm * 0.03, 3),
    max_amp_rad = k.max_rpm * 0.6,
    skip_cycles = 2,
    cycles = 4,
    timeout_s = 5,
    min_period_s = 0.02,
    max_period_s = 2,
    max_spread = 0.35,
  })
end

local function tune_start_direction(dir)
  tune.dir = dir
  tune.state = "ff"
  tune.t = 0
  tune.sum_current = { 0, 0 }
  tune.sum_rpm = { 0, 0 }
  tune.count = 0
  tune.base = { 0, 0 }
  tune.relays = nil
end

local function tune_finish()
  local p_sum, i_sum, ff_sum, n = 0, 0, 0, 0
  local p_dir = { 0, 0 }
  for d = 1, 2 do
    for s = 1, 2 do
      local r = tune.results[d][s]
      if not finite(r.P) or not finite(r.I) or r.P <= 0 or r.I <= 0 then
        tune_fail("invalid gains")
        return
      end
      p_sum = p_sum + r.P
      i_sum = i_sum + r.I
      ff_sum = ff_sum + r.ff
      p_dir[d] = p_dir[d] + r.P
      n = n + 1
    end
  end
  local kp, ki, kff = p_sum / n, i_sum / n, ff_sum / n
  if p_dir[1] > 1.3 * p_dir[2] or p_dir[2] > 1.3 * p_dir[1] then
    tune_message(MAV_SEVERITY.WARNING, string.format("forward/reverse differ (P %.4f / %.4f)",
      p_dir[1] / 2, p_dir[2] / 2))
  end
  M.queue_save(P.FF, kff)
  M.queue_save(P.P, kp)
  M.queue_save(P.I, ki)
  M.queue_save(P.MODE, 0)
  M.queue_save(scr_user1, 0)
  side[1].integrator, side[2].integrator = 0, 0
  tune.state = "done"
  tune_message(MAV_SEVERITY.NOTICE, string.format("saved ESC_FF=%.4f ESC_P=%.4f ESC_I=%.4f", kff, kp, ki))
  tune_message(MAV_SEVERITY.NOTICE, "SCR_USER1 set to 0 (normal)")
end

local function tune_log(s, target, current)
  local relay = tune.relays and tune.relays[s]
  logger:write("DJWT", "Side,Dir,Ph,Tgt,Rpm,Cur,Cyc", "BbBfffB",
    s, tune.dir or 0, tune.state == "relay" and 2 or 1, target, side[s].rpm, current,
    relay and relay.cycle or 0)
end

--[[
Wheel autotune state machine. Returns per-side currents while it owns the
outputs, or nil to let the normal speed loop run.
Phase "ff":    side PI holds dir * ESC_WT_RPM; average current gives ESC_FF.
Phase "relay": current = I_base +/- relay around the same reference (Astrom-Hagglund).
Forward first, then reverse to come back.
--]]
local function wheel_tune(dt, k, armed, enabled)
  local wanted = tune_wanted(armed)
  if not wanted then
    if tune.state == "ff" or tune.state == "relay" then
      tune_message(MAV_SEVERITY.WARNING, "aborted (left HOLD, disarmed or selector changed)")
    end
    if tune.state ~= "idle" then
      tune_reset()
      side[1].integrator, side[2].integrator = 0, 0
    end
    return nil
  end
  if tune.state == "done" or tune.state == "fail" then
    return nil
  end
  if not k.speed_mode then
    if tune.state == "idle" then
      tune_fail("ESC_MODE must be 0")
    end
    return nil
  end
  if tune.state == "idle" then
    if not enabled then
      return nil
    end
    if not rc_centered() then
      local now = now_ms()
      if now - last_tune_warn >= 5000 then
        last_tune_warn = now
        tune_message(MAV_SEVERITY.WARNING, "center the sticks to start")
      end
      return nil
    end
    tune.results = {}
    tune_start_direction(1)
    tune_message(MAV_SEVERITY.NOTICE, string.format("started; %.0f rpm forward then reverse, relay %.1f A",
      k.max_rpm * constrain(g("WT_RPM"), 5, 50) / 100,
      constrain(k.max_current * constrain(g("WT_RELAY"), 1, 80) / 100, 0.5, 5)))
  end
  if not enabled then
    tune_fail("ESC feedback lost or outputs disabled")
    return nil
  end
  if not rc_centered() then
    tune_fail("stick moved")
    return nil
  end

  local target = tune.dir * k.max_rpm * constrain(g("WT_RPM"), 5, 50) / 100
  local currents = { 0, 0 }
  tune.t = tune.t + dt

  if tune.state == "ff" then
    for s = 1, 2 do
      profile_reference(s, target, dt)
      currents[s] = side_pi(s, side[s].ref_rpm, dt, k)
      if tune.t >= K.TUNE_SETTLE_S then
        tune.sum_current[s] = tune.sum_current[s] + currents[s]
        tune.sum_rpm[s] = tune.sum_rpm[s] + side[s].rpm
      end
    end
    if tune.t >= K.TUNE_SETTLE_S then
      tune.count = tune.count + 1
    end
    if tune.t >= K.TUNE_SETTLE_S + K.TUNE_AVERAGE_S then
      tune.relays = {}
      for s = 1, 2 do
        local mean_rpm = tune.sum_rpm[s] / tune.count
        if math.abs(mean_rpm - target) > 0.3 * math.abs(target) then
          tune_fail(string.format("%s side did not reach %.0f rpm (%.0f)", s == 1 and "left" or "right",
            target, mean_rpm))
          return { 0, 0 }
        end
        tune.base[s] = tune.sum_current[s] / tune.count
        tune.relays[s] = make_relay(k)
        tune.relays[s]:start()
      end
      tune.state = "relay"
      tune.t = 0
    end
  elseif tune.state == "relay" then
    local finished = true
    for s = 1, 2 do
      local relay = tune.relays[s]
      local u = 0
      if relay.status == "running" then
        local status
        u, status = relay:step(target - side[s].rpm, dt)
        if status == "fail" then
          tune_fail(string.format("%s side: %s", s == 1 and "left" or "right", relay:result().msg))
          return { 0, 0 }
        end
      end
      if relay.status ~= "done" then
        finished = false
      end
      currents[s] = constrain(tune.base[s] + u, -k.max_current, k.max_current)
    end
    if finished then
      local results = {}
      for s = 1, 2 do
        local r = tune.relays[s]:result()
        r.ff = math.abs(tune.base[s]) / math.abs(target)
        results[s] = r
        -- Bumpless return to the speed loop
        side[s].integrator = constrain(tune.base[s] - k.kff * target, -k.imax, k.imax)
        side[s].last_error = 0
      end
      tune.results[#tune.results + 1] = results
      tune_message(MAV_SEVERITY.INFO, string.format("%s: Ku %.4f/%.4f Pu %.3f/%.3f s",
        tune.dir > 0 and "forward" or "reverse", results[1].Ku, results[2].Ku, results[1].Pu, results[2].Pu))
      if tune.dir > 0 then
        tune_start_direction(-1)
      else
        tune_finish()
        return { 0, 0 }
      end
    end
  end
  if logger ~= nil then
    tune_log(1, target, currents[1])
    tune_log(2, target, currents[2])
  end
  return currents
end

--[[
Per-wheel tuning (SCR_USER1 = 5 breakaway, 6 local relay). LT.step returns the
current of every wheel while it owns the outputs, or nil to let the normal loop run.
--]]
local LT = { state = "idle" }

function LT.fail(msg)
  LT.state = "fail"
  tune_message(MAV_SEVERITY.ERROR, "failed - " .. msg .. "; parameters kept")
end

function LT.done(text)
  LT.state = "done"
  M.queue_save(scr_user1, 0)
  tune_message(MAV_SEVERITY.NOTICE, text)
  tune_message(MAV_SEVERITY.NOTICE, "SCR_USER1 set to 0 (normal)")
end

-- The local relay needs exactly two wheels on each side
function LT.find_pairs()
  LT.wheels = { {}, {} }
  for i = 0, K.MAX_FB - 1 do
    if esc[i + 1].detected then
      local w = LT.wheels[side_of(i)]
      w[#w + 1] = i
    end
  end
  return #LT.wheels[1] == 2 and #LT.wheels[2] == 2
end

function LT.start(dir)
  LT.dir = dir
  LT.t = 0
  if LT.mode == K.BRK_SELECTOR then
    LT.state = "brk"
    LT.found = {}
    LT.moving = {}
  else
    LT.state = "hold"
    LT.relays = nil
  end
end

function LT.breakaway(dt, k)
  local cur = { 0, 0, 0, 0, 0, 0, 0, 0 }
  if LT.state == "rest" then
    if LT.t >= K.BRK_REST_S then
      LT.start(-1)
    end
    return cur
  end
  local amps = LT.t * K.BRK_RAMP
  local limit = k.max_current * K.BRK_MAX_FRAC
  local pending = false
  for i = 0, K.NUM_ESC - 1 do
    local fb = esc[i + 1]
    if fb.detected and LT.found[i + 1] == nil then
      if LT.dir * fb.wheel_rpm > K.BRK_RPM then
        LT.moving[i + 1] = (LT.moving[i + 1] or 0) + dt
      else
        LT.moving[i + 1] = 0
      end
      if LT.moving[i + 1] >= K.BRK_CONFIRM_S then
        LT.found[i + 1] = math.max(amps - K.BRK_RAMP * K.BRK_CONFIRM_S, 0)
      elseif amps > limit then
        LT.fail(string.format("ESC %d did not turn up to %.1f A; wheels in the air?", i + 1, limit))
        return nil
      else
        pending = true
        cur[i + 1] = LT.dir * amps
      end
    end
  end
  if pending then
    return cur
  end
  local f = LT.found
  LT.results[#LT.results + 1] = f
  tune_message(MAV_SEVERITY.INFO, string.format("%s breakaway %.2f %.2f %.2f %.2f A",
    LT.dir > 0 and "forward" or "reverse", f[1] or 0, f[2] or 0, f[3] or 0, f[4] or 0))
  if LT.dir > 0 then
    LT.state = "rest"
    LT.t = 0
    return cur
  end
  local fr = {}
  for i = 1, K.NUM_ESC do
    if LT.results[1][i] ~= nil and LT.results[2][i] ~= nil then
      fr[i] = (LT.results[1][i] + LT.results[2][i]) / 2
      M.queue_save(P["FR" .. i], fr[i])
    end
  end
  -- ESC_LOC_MAX covers twice the worst friction mismatch inside one side
  local worst = 0
  for i = 0, K.NUM_ESC - 1 do
    if fr[i + 1] ~= nil then
      local sum, n = 0, 0
      for j = 0, K.NUM_ESC - 1 do
        if fr[j + 1] ~= nil and side_of(j) == side_of(i) then
          sum, n = sum + fr[j + 1], n + 1
        end
      end
      worst = math.max(worst, math.abs(fr[i + 1] - sum / n))
    end
  end
  local pct = constrain(math.ceil(200 * worst / math.max(k.max_current, 0.1)), 10, 50)
  M.queue_save(P.LOC_MAX, pct)
  LT.done(string.format("saved ESC_FR %.2f %.2f %.2f %.2f A, ESC_LOC_MAX=%d",
    fr[1] or 0, fr[2] or 0, fr[3] or 0, fr[4] or 0, pct))
  return nil
end

function LT.relay(dt, k)
  local target = LT.dir * k.max_rpm * constrain(g("WT_RPM"), 5, 50) / 100
  local base = { 0, 0 }
  for s = 1, 2 do
    profile_reference(s, target, dt)
    base[s] = side_pi(s, side[s].ref_rpm, dt, k)
  end
  local u = { 0, 0 }
  if LT.state == "hold" then
    if LT.t >= K.TUNE_SETTLE_S then
      LT.relays = {}
      for s = 1, 2 do
        if math.abs(side[s].rpm - target) > 0.3 * math.abs(target) then
          LT.fail(string.format("%s side did not reach %.0f rpm (%.0f)", s == 1 and "left" or "right",
            target, side[s].rpm))
          return nil
        end
        -- The wheel-to-wheel difference moves twice as fast as one side, so twice the side hysteresis
        LT.relays[s] = make_relay(k, math.max(k.max_rpm * 0.06, 6))
        LT.relays[s]:start()
      end
      LT.state = "relay"
      LT.t = 0
    end
  else
    local finished = true
    for s = 1, 2 do
      local relay = LT.relays[s]
      if relay.status == "running" then
        local a, b = LT.wheels[s][1], LT.wheels[s][2]
        local status
        u[s], status = relay:step(esc[b + 1].wheel_rpm - esc[a + 1].wheel_rpm, dt)
        if status == "fail" then
          LT.fail(string.format("%s side: %s", s == 1 and "left" or "right", relay:result().msg))
          return nil
        end
      end
      if relay.status ~= "done" then
        finished = false
      end
    end
    if finished then
      local res = { LT.relays[1]:result(), LT.relays[2]:result() }
      LT.results[#LT.results + 1] = res
      tune_message(MAV_SEVERITY.INFO, string.format("local %s: Ku %.4f/%.4f Pu %.3f/%.3f s",
        LT.dir > 0 and "forward" or "reverse", res[1].Ku, res[2].Ku, res[1].Pu, res[2].Pu))
      if LT.dir > 0 then
        LT.start(-1)
      else
        local p_sum, i_sum, n = 0, 0, 0
        for _, r2 in ipairs(LT.results) do
          for s = 1, 2 do
            local r = r2[s]
            if not finite(r.P) or not finite(r.I) or r.P <= 0 or r.I <= 0 then
              LT.fail("invalid gains")
              return nil
            end
            p_sum, i_sum, n = p_sum + r.P, i_sum + r.I, n + 1
          end
        end
        -- The relay acts on (right wheel - left wheel) of a pair; the local term acts on
        -- (side mean - wheel), which is half of that difference, so the gains double.
        local lp, li = 2 * p_sum / n, 2 * i_sum / n
        M.queue_save(P.LOC_P, lp)
        M.queue_save(P.LOC_I, li)
        LT.done(string.format("saved ESC_LOC_P=%.4f ESC_LOC_I=%.4f", lp, li))
        return nil
      end
    end
  end
  local cur = { 0, 0, 0, 0, 0, 0, 0, 0 }
  for s = 1, 2 do
    local a, b = LT.wheels[s][1], LT.wheels[s][2]
    cur[a + 1] = base[s] + u[s]
    cur[b + 1] = base[s] - u[s]
  end
  return cur
end

function LT.step(dt, k, armed, enabled)
  local sel = selector()
  local mode = nil
  if sel == K.BRK_SELECTOR or sel == K.LOC_SELECTOR then
    mode = sel
  end
  local running = LT.state == "brk" or LT.state == "rest" or LT.state == "hold" or LT.state == "relay"
  local wanted = mode ~= nil and armed and vehicle:get_mode() == K.HOLD_MODE and
    (LT.state == "idle" or LT.mode == mode)
  if not wanted then
    if running then
      tune_message(MAV_SEVERITY.WARNING, "aborted (left HOLD, disarmed or selector changed)")
    end
    LT.state = "idle"
    return nil
  end
  if LT.state == "done" or LT.state == "fail" then
    return nil
  end
  if LT.state == "idle" then
    local now = now_ms()
    if not enabled then
      if now - last_tune_warn >= 5000 then
        last_tune_warn = now
        if not latched_can then
          tune_message(MAV_SEVERITY.WARNING, "waiting: ESC_CAN is not latched")
        elseif SRV_Channels:get_safety_state() then
          tune_message(MAV_SEVERITY.WARNING, "waiting: disable safety switch")
        elseif SRV_Channels:get_emergency_stop() then
          tune_message(MAV_SEVERITY.WARNING, "waiting: emergency stop is on")
        elseif not mask_valid() then
          tune_message(MAV_SEVERITY.WARNING, "waiting: ESC_LMASK invalid")
        elseif not all_healthy() then
          tune_message(MAV_SEVERITY.WARNING, "waiting: not all ESCs healthy (found ESC messages?)")
        else
          tune_message(MAV_SEVERITY.WARNING, "waiting: outputs not enabled yet")
        end
      end
      return nil
    end
    if not rc_centered() then
      if now - last_tune_warn >= 5000 then
        last_tune_warn = now
        tune_message(MAV_SEVERITY.WARNING, "center the sticks to start")
      end
      return nil
    end
    LT.mode = mode
    LT.results = {}
    if mode == K.LOC_SELECTOR then
      if not k.speed_mode then
        LT.fail("ESC_MODE must be 0")
        return nil
      end
      if not LT.find_pairs() then
        LT.fail("needs two ESCs on each side")
        return nil
      end
      tune_message(MAV_SEVERITY.NOTICE, "local relay started; forward then reverse")
    else
      tune_message(MAV_SEVERITY.NOTICE, "breakaway started; wheels must be in the air")
    end
    LT.start(1)
  end
  if not enabled then
    LT.fail("ESC feedback lost or outputs disabled")
    return nil
  end
  if not rc_centered() then
    LT.fail("stick moved")
    return nil
  end
  LT.t = LT.t + dt
  if LT.mode == K.BRK_SELECTOR then
    return LT.breakaway(dt, k)
  end
  return LT.relay(dt, k)
end

local function check_slip(k, now, enabled)
  local any = false
  for i = 0, K.MAX_FB - 1 do
    local fb = esc[i + 1]
    local sd = side[side_of(i)]
    local diff = math.abs(fb.wheel_rpm - sd.rpm_mean)
    if enabled and k.slip > 0 and diff > k.slip then
      slip_since[i + 1] = slip_since[i + 1] or now
      if now - slip_since[i + 1] >= K.SLIP_MS then
        any = true
        if now - last_slip_warn >= 5000 then
          last_slip_warn = now
          gcs:send_text(MAV_SEVERITY.WARNING, string.format("ESC: slip on ESC %d (%.0f rpm from its side)", i + 1, diff))
        end
      end
    else
      slip_since[i + 1] = nil
    end
  end
  slip_active = any
end

local function run_control()
  local now_us = micros()
  local dt = 0.01
  if last_us ~= nil then
    dt = constrain(as_number(now_us - last_us) * 1.0e-6, 0, 0.05)
  end
  last_us = now_us
  local now = now_ms()

  local k = read_gains()
  local hold_disarmed = (k.opt & 2) ~= 0
  local armed = arming:is_armed()
  local safety_live = not SRV_Channels:get_safety_state()
  local estop = SRV_Channels:get_emergency_stop()
  local outputs_ok = latched_can and safety_live and not estop and mask_valid() and all_healthy()
  local enabled = outputs_ok and (armed or (k.speed_mode and hold_disarmed))

  -- Feedback per wheel and per side.
  -- rpm_mean: arithmetic mean (logging, slip warning, local equalization when synced).
  -- rpm_lag:  slowest wheel in the commanded direction. When the two wheels of a side
  -- diverge, side PI uses rpm_lag so a free-spinning wheel cannot satisfy the target
  -- while the grounded wheel is still stuck.
  local sum = { 0, 0 }
  local count = { 0, 0 }
  local lag = { nil, nil }
  local fast = { nil, nil }
  for i = 0, K.MAX_FB - 1 do
    local fb = esc[i + 1]
    fb.wheel_rpm = direction(i) * fb.rotor_rpm / k.gear
    if fb.detected then
      local s = side_of(i)
      sum[s] = sum[s] + fb.wheel_rpm
      count[s] = count[s] + 1
      local cmd = side[s].interp_value
      local progress = (cmd >= 0) and fb.wheel_rpm or -fb.wheel_rpm
      if lag[s] == nil or progress < lag[s] then
        lag[s] = progress
      end
      if fast[s] == nil or progress > fast[s] then
        fast[s] = progress
      end
    end
  end
  -- Divergence threshold: ESC_SLIP when set, otherwise 10% of ESC_RPM (min 40).
  local diverge_rpm = k.slip > 0 and k.slip or math.max(0.1 * k.max_rpm, 40)
  for s = 1, 2 do
    local sd = side[s]
    if count[s] > 0 then
      sd.rpm_mean = sum[s] / count[s]
      local sgn = (sd.interp_value >= 0) and 1 or -1
      sd.rpm_lag = (lag[s] ~= nil) and (lag[s] * sgn) or sd.rpm_mean
      sd.diverge_rpm = diverge_rpm
      sd.diverged = count[s] >= 2 and fast[s] ~= nil and lag[s] ~= nil and
        (fast[s] - lag[s]) > diverge_rpm and math.abs(sd.interp_value) >= 0.01
      -- Autotune paths keep the mean; normal speed mode swaps to lag below.
      sd.rpm = sd.rpm_mean
    else
      sd.rpm_mean = 0
      sd.rpm_lag = 0
      sd.diverge_rpm = diverge_rpm
      sd.diverged = false
      sd.rpm = 0
    end
  end

  update_command(now, dt)
  if not armed then
    side[1].interp_value, side[2].interp_value = 0, 0
    side[1].goal, side[2].goal = 0, 0
  end

  local currents = nil
  local wheel_currents = nil
  local live = enabled
  currents = wheel_tune(dt, k, armed, live)
  if not live then
    currents = nil
  end
  if currents == nil then
    wheel_currents = LT.step(dt, k, armed, live)
    if not live then
      wheel_currents = nil
    end
  end

  local raw = { 0, 0, 0, 0, 0, 0, 0, 0 }
  if not live then
    reset_control()
  elseif wheel_currents ~= nil then
    for i = 0, K.MAX_FB - 1 do
      local fb = esc[i + 1]
      if fb.detected then
        fb.current_a = constrain(wheel_currents[i + 1], -k.max_current, k.max_current)
        raw[i + 1] = direction(i) * fb.current_a * K.RAW_PER_AMP
      end
    end
  else
    local tuning = currents ~= nil
    if not tuning then
      currents = { 0, 0 }
      if k.speed_mode then
        local targets = {}
        for s = 1, 2 do
          profile_reference(s, side[s].interp_value * k.max_rpm, dt)
          targets[s] = side[s].ref_rpm
          -- Use the lagging wheel when the pair has diverged so the free spinner
          -- cannot hold the side PI at the target while the other wheel is stuck.
          if side[s].diverged then
            side[s].rpm = side[s].rpm_lag
          end
        end
        -- Desaturation: a saturated side cannot reach its reference, so scale the
        -- other side by the same achieved fraction to keep the turn radius.
        if (k.opt & 4) ~= 0 then
          for s = 1, 2 do
            local o = 3 - s
            if side[s].saturated and not side[o].saturated and math.abs(targets[s]) > K.MOVING_RPM then
              targets[o] = targets[o] * constrain(side[s].rpm / targets[s], 0, 1)
            end
          end
        end
        for s = 1, 2 do
          currents[s] = side_pi(s, targets[s], dt, k)
        end
      else
        for s = 1, 2 do
          side[s].target_rpm = 0
          side[s].integrator = 0
          side[s].saturated = false
          currents[s] = constrain(side[s].interp_value * k.max_current, -k.max_current, k.max_current)
        end
      end
    end
    if side[1].saturated or side[2].saturated then
      sat_ms = now
    end
    -- Friction compensation only on a side where every wheel has a measured ESC_FRn
    local fr_sum, fr_n, fr_ok = { 0, 0 }, { 0, 0 }, { true, true }
    for i = 0, K.MAX_FB - 1 do
      if esc[i + 1].detected then
        local s = side_of(i)
        local fr = k.fr[i + 1]
        if fr == nil or fr <= 0 then
          fr_ok[s] = false
        else
          fr_sum[s], fr_n[s] = fr_sum[s] + fr, fr_n[s] + 1
        end
      end
    end
    for i = 0, K.MAX_FB - 1 do
      local fb = esc[i + 1]
      if fb.detected then
        local s = side_of(i)
        local sd = side[s]
        local demand = currents[s]
        local ref = k.speed_mode and sd.ref_rpm or sd.interp_value * k.max_rpm
        local cmd_sgn = (ref >= 0) and 1 or -1
        -- Synced: equalize to the mean (zero-sum). Diverged: drive the lagging wheel
        -- to the side reference; the spinning wheel is starved below.
        local loc_err = sd.diverged and (ref - fb.wheel_rpm) or (sd.rpm_mean - fb.wheel_rpm)
        if tuning or k.loc_i <= 0 or math.abs(sd.interp_value) < 0.01 then
          fb.loc_int = 0
        else
          fb.loc_int = constrain(fb.loc_int + k.loc_i * loc_err * dt, -k.loc_max, k.loc_max)
        end
        local comp = 0
        if not tuning and fr_ok[s] and fr_n[s] > 0 then
          comp = constrain(ref / K.MOVING_RPM, -1, 1) * (k.fr[i + 1] - fr_sum[s] / fr_n[s])
        end
        if not tuning and (k.loc_p > 0 or k.loc_i > 0 or comp ~= 0) then
          demand = demand + constrain(k.loc_p * loc_err + fb.loc_int + comp, -k.loc_max, k.loc_max)
        end
        -- Traction split: with one wheel in the air the shared side current would only
        -- accelerate the free spinner. Cut drive on the fast wheel and keep full demand
        -- on the lagging (ground) wheel so the rover can yaw/move.
        if not tuning and sd.diverged and k.speed_mode and math.abs(ref) > K.MOVING_RPM then
          local progress = fb.wheel_rpm * cmd_sgn
          local lag_progress = sd.rpm_lag * cmd_sgn
          local is_fast = progress > lag_progress + 0.5 * (sd.diverge_rpm or 40)
          if is_fast then
            local brake = k.brake and k.brake_current or 0
            -- No drive in the command direction; optional light brake if overspeeding.
            if progress > math.abs(ref) then
              demand = -cmd_sgn * brake
            else
              demand = 0
            end
          else
            -- Ground wheel: full side current toward the target (already raised by lag PI).
            demand = currents[s]
            if k.loc_p > 0 or k.loc_i > 0 or comp ~= 0 then
              demand = demand + constrain(k.loc_p * (ref - fb.wheel_rpm) + fb.loc_int + comp,
                -k.loc_max, k.loc_max)
            end
            -- Still stuck near zero: push hard up to ESC_CUR so breakaway can happen.
            if math.abs(fb.wheel_rpm) < K.MOVING_RPM then
              local push = cmd_sgn * k.max_current
              if demand * cmd_sgn < push * cmd_sgn then
                demand = push
              end
            end
          end
        end
        local lo, hi = current_window(fb.wheel_rpm, k)
        fb.current_a = constrain(demand, lo, hi)
        sd.current = currents[s]
        raw[i + 1] = direction(i) * fb.current_a * K.RAW_PER_AMP
      end
    end
  end
  check_slip(k, now, enabled)

  -- PWM mode leaves the bus silent so the C620 can follow the servo pulse.
  if not latched_can then
    return
  end
  local low, high = false, false
  for i = 1, 4 do
    if esc[i].detected then
      low = true
    end
    if esc[i + 4].detected then
      high = true
    end
  end
  if low then
    send_group(0x200, raw, 1)
  end
  if high then
    send_group(0x1FF, raw, 5)
  end
end

local function check_health()
  if not latched_can or not arming:is_armed() then
    lost_mask = 0
    return
  end
  local lost = 0
  for i = 0, K.MAX_FB - 1 do
    if esc[i + 1].detected and not healthy(i) then
      lost = lost | (1 << i)
    end
  end
  local now = now_ms()
  if lost ~= 0 and (lost ~= lost_mask or now - last_lost_warn >= 5000) then
    last_lost_warn = now
    for i = 0, K.MAX_FB - 1 do
      if (lost & (1 << i)) ~= 0 then
        gcs:send_text(MAV_SEVERITY.CRITICAL, string.format("ESC: ESC %d lost, all motors stopped", i + 1))
      end
    end
  end
  if lost == 0 and lost_mask ~= 0 then
    gcs:send_text(MAV_SEVERITY.INFO, "ESC: feedback restored")
  end
  lost_mask = lost
end

local function prearm()
  if auth_id == nil then
    return
  end
  if not latched_can then
    arming:set_aux_auth_passed(auth_id)
    return
  end
  if param:get('C620_CAN') ~= nil then
    arming:set_aux_auth_failed(auth_id, "delete c620_can.lua from APM/scripts")
    return
  end
  if not mask_valid() then
    arming:set_aux_auth_failed(auth_id, "ESC_LMASK needs ESCs on both sides")
    return
  end
  local saw = false
  for i = 0, K.MAX_FB - 1 do
    if esc[i + 1].detected then
      saw = true
    end
    if is_throttle(i) and not esc[i + 1].detected then
      arming:set_aux_auth_failed(auth_id, string.format("ESC %d not found (SERVO%d is a motor)", i + 1, i + 1))
      return
    end
    if is_throttle(i) and not healthy(i) then
      arming:set_aux_auth_failed(auth_id, string.format("ESC %d no feedback", i + 1))
      return
    end
  end
  if not saw then
    arming:set_aux_auth_failed(auth_id, "no ESC detected")
    return
  end
  arming:set_aux_auth_passed(auth_id)
end

local function publish_stat()
  local stat = 0
  if latched_can and driver ~= nil then
    stat = stat | K.STAT_CAN
  end
  if sat_ms ~= nil and now_ms() - sat_ms < K.SAT_HOLD_MS then
    stat = stat | K.STAT_SAT
  end
  if tune.state == "ff" or tune.state == "relay" or
    LT.state == "brk" or LT.state == "rest" or LT.state == "hold" or LT.state == "relay" then
    stat = stat | K.STAT_TUNE_RUN
  elseif tune.state == "done" or LT.state == "done" then
    stat = stat | K.STAT_TUNE_OK
  elseif tune.state == "fail" or LT.state == "fail" then
    stat = stat | K.STAT_TUNE_FAIL
  end
  if slip_active then
    stat = stat | K.STAT_SLIP
  end
  if stat ~= published_stat then
    published_stat = stat
    set_param("STAT", stat)
  end
end

local function write_log()
  if (math.floor(g("OPT")) & 1) == 0 or logger == nil then
    return
  end
  local now = now_ms()
  if now - last_log < 50 then
    return
  end
  last_log = now
  local gear = math.max(g("GEAR"), 1)
  for i = 0, K.NUM_ESC - 1 do
    local fb = esc[i + 1]
    if fb.detected then
      local dir = direction(i)
      local sd = side[side_of(i)]
      logger:write("DJIC", "Id,Tgt,Rpm,CurD,CurM,I,Pos,Temp", "BffffffB",
        i + 1,
        sd.target_rpm,
        fb.wheel_rpm,
        fb.current_a,
        dir * fb.current_raw / K.RAW_PER_AMP,
        sd.integrator,
        dir * fb.rotor_counts / (K.COUNTS_PER_REV * gear),
        fb.temperature)
    end
  end
end

local function publish_rpm(idx, rpm)
  local ok = pcall(function()
    esc_telem:update_rpm(idx, rpm, 0)
  end)
  if ok or rpm >= 0 then
    return ok
  end
  -- Older firmware stores this RPM as uint16 and rejects a negative value.
  return pcall(function()
    esc_telem:update_rpm(idx, -rpm, 0)
  end)
end

local function send_telem()
  if esc_telem == nil or telem_slots <= 0 or now_ms() - last_telem < 20 then
    return
  end
  last_telem = now_ms()
  for i = 0, telem_slots - 1 do
    local fb = esc[i + 1]
    if fb.detected and healthy(i) then
      local rpm = direction(i) * fb.rotor_rpm
      if not publish_rpm(i, rpm) then
        telem_slots = i
        if not telem_warned then
          telem_warned = true
          gcs:send_text(MAV_SEVERITY.WARNING, string.format("ESC: telemetry limited to %d", i))
        end
        return
      end
      pcall(function()
        local data = ESCTelemetryData()
        data:motor_temp_cdeg(math.floor(constrain(fb.temperature, 0, 200) * 100))
        data:current(math.abs(fb.current_raw) / K.RAW_PER_AMP)
        esc_telem:update_telem_data(i, data, K.TELEM_MOTOR_TEMP | K.TELEM_CURRENT)
      end)
    end
  end
end

-- While a motor ESC is missing, report the frames seen per ID every 5 s so a
-- silent ESC (PWM mode, wrong ID, wiring) can be told apart from a slow loop
local function report_rx()
  local now = now_ms()
  if diag.last_loop_ms ~= 0 then
    diag.max_dt = math.max(diag.max_dt, now - diag.last_loop_ms)
  end
  diag.last_loop_ms = now
  if now - diag.last_ms < 5000 then
    return
  end
  local missing = false
  for i = 0, K.MAX_FB - 1 do
    if is_throttle(i) and not esc[i + 1].detected then
      missing = true
    end
  end
  if latched_can and missing and not arming:is_armed() then
    local n = diag.n
    gcs:send_text(MAV_SEVERITY.WARNING, string.format("ESC rx %d %d %d %d|%d x%d@%X dt%d",
      n[1], n[2], n[3], n[4], n[5] + n[6] + n[7] + n[8], diag.other, diag.other_id, diag.max_dt))
  end
  for i = 1, K.MAX_FB do
    diag.n[i] = 0
  end
  diag.other = 0
  diag.max_dt = 0
  diag.last_ms = now
end

local function warn_selector_without_can()
  local sel = selector()
  if latched_can or (sel ~= K.WHEEL_TUNE_SELECTOR and sel ~= K.BRK_SELECTOR and sel ~= K.LOC_SELECTOR) then
    return
  end
  local now = now_ms()
  if now - last_tune_warn >= 10000 then
    last_tune_warn = now
    tune_message(MAV_SEVERITY.WARNING, string.format("SCR_USER1=%d needs ESC_CAN=1; nothing done", math.floor(sel)))
  end
end

local function update()
  if not ensure_params() then
    return update, 1000
  end
  if auth_id == nil then
    auth_id = arming:get_aux_auth_id()
  end
  if driver == nil then
    driver = CAN:get_device(25)
    if driver == nil then
      local want_can = g("CAN") >= 0.5
      local now = now_ms()
      if want_can and now - last_driver_warn >= 5000 then
        last_driver_warn = now
        gcs:send_text(MAV_SEVERITY.WARNING, "ESC: ESC_CAN=1 but no scripting CAN driver (CAN_Dx_PROTOCOL=10)")
      end
      -- Refuse to arm instead of silently driving on PWM when CAN was asked for
      if auth_id ~= nil then
        if want_can then
          arming:set_aux_auth_failed(auth_id, "ESC_CAN=1 but CAN driver missing")
        else
          arming:set_aux_auth_passed(auth_id)
        end
      end
      publish_stat()
      return update, 1000
    end
    if not announced_driver then
      announced_driver = true
      gcs:send_text(MAV_SEVERITY.INFO, "ESC: Lua CAN driver ready")
      -- The scripting driver takes only the first port mapped to it; the other is ignored
      local p1 = param:get('CAN_P1_DRIVER') or 0
      local p2 = param:get('CAN_P2_DRIVER') or 0
      if p1 ~= 0 and p1 == p2 then
        gcs:send_text(MAV_SEVERITY.WARNING, "ESC: CAN_P1/P2_DRIVER both " .. math.floor(p1) .. ", only CAN1 used")
      end
    end
  end

  M.refresh_maps(false)
  apply_switch()
  if latched_can and not mask_valid() then
    local now = now_ms()
    if now - last_mask_warn >= 5000 then
      last_mask_warn = now
      gcs:send_text(MAV_SEVERITY.ERROR, "ESC: ESC_LMASK must put ESCs on both sides")
    end
  end
  -- One kill per loop is enough; timeout 3000 ms covers the 10 ms period
  if latched_can then
    kill_pwm()
  end
  drain()
  run_control()
  if latched_can then
    check_health()
  end
  warn_selector_without_can()
  report_rx()
  publish_stat()
  prearm()
  -- Spread EEPROM / telem / log so a single update stays under SCR_VM_I_COUNT
  M.drain_saves()
  local tick = now_ms() % 100
  if tick < 50 then
    send_telem()
  else
    write_log()
  end
  return update, K.LOOP_MS
end

return update, 2000
