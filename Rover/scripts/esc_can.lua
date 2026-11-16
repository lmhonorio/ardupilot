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
-- ESC_MODE     0 = per-wheel speed PI, 1 = current follows the command
-- ESC_RPM      wheel rpm at command 1.0
-- ESC_GEAR     rotor-to-wheel ratio (M3508 P19 = 19.2032)
-- ESC_CUR      current limit, ampere
-- ESC_P _I _D _FF _IMAX   shared speed PI, used when that wheel's own gain is 0.
--                        Current in A, error in wheel rpm. Each wheel has its own
--                        integrator and tracks the rpm reference of its side.
-- ESC_P1..4 _I1..4 _FF1..4  per-wheel PI and rpm feed-forward. 0 = use ESC_P / ESC_I / ESC_FF.
-- ESC_REV      bitmask, bit 0 = ESC1 ... reverses that motor
-- ESC_OPT      bit 0 log DJIC, bit 1 hold zero rpm while disarmed,
--              bit 2 desaturation (keep the left/right ratio when a side saturates)
-- ESC_BRK      0 = never push current against the spin, 1 = brake up to ESC_BRKCUR
-- ESC_ACC      reference acceleration limit, wheel rpm/s, both ways (0 = none = fully reactive).
--              Keep at 0 for stick/mission command with no artificial ramp.
-- ESC_BRKCUR   braking current limit, % of ESC_CUR
-- ESC_LOC_*    unused. The old side correction was replaced by the per-wheel PI.
-- ESC_FR1..4   unused. Kept so an existing parameter table does not shift.
-- ESC_SLIP     wheel-to-side rpm difference reported as slip (0 = off)
-- ESC_LMASK    bitmask of the ESCs (1..4) on the left side; the others are right
--              (ESC_LMASK=9 → ESC1+ESC4 left, ESC2+ESC3 right)
-- ESC_WT_RELAY wheel autotune current, % of ESC_CUR (1..100): the current step
--              of USER1=4 and 5. 20-30% suits the loaded rover (a step that moves
--              it but stays below the rpm guard); less is enough in the air.
-- ESC_WT_BW    USER1=4 target bandwidth, rad/s (default 8). Higher = faster and
--              stiffer with bigger current peaks; capped by the measured lag.
-- ESC_WT_RPM   high plateau of USER1=4 and 5, wheel rpm. The low plateau is half.
--              Values below 60 stay on 120, so an old saved 30 does not take over.
-- ESC_GCS_HZ   live NAMED_VALUE_FLOAT stream to the GCS, Hz (0 = off, max 20):
--              ESCn_I measured current A, ESCn_ICMD commanded current A,
--              ESCn_RPM wheel rpm, ESC_TGT_L / ESC_TGT_R side target rpm.
-- ESC_PREF     reference prefilter on the wheel speed PI: time constant as a
--              fraction of that wheel's Ti (= Kp/Ki). 0 = off, default 0.5.
--              The tuned PI (symmetrical optimum) overshoots a speed step
--              (about 40% seen on the rover). In simulation with the tuned
--              gains: 0.5 -> 4% overshoot, +0.09 s; 1.0 -> 0%, +0.28 s.
--              Only speeding up is filtered: slowing and stopping are immediate.
--
-- Both wheels of a side share the side command as their speed target. Each closes
-- its own PI on its own encoder, so a slipping wheel cannot satisfy the loop of
-- the wheel that is still on the ground.
--
-- SCR_USER1 = 4, armed in HOLD, on the ground, a few metres free. Every wheel
-- holds 60 then 120 rpm (ESC_FFn = current slope between them), then gets a
-- current step up and down at 120 rpm: the rpm slope is Kv with the rover's
-- own mass and the line intercept is the lag T. Kp = wc / Kv, Ti = 4 / wc with
-- wc = min(ESC_WT_BW, 1 / (2.65 T)). A closed-loop check (60 -> 120 rpm with
-- ESC_ACC and ESC_PREF) must stay under 15% overshoot, or wc drops by 25%
-- (3 tries). Forward then reverse; P and I are per wheel.
-- SCR_USER1 = 5, WHEELS IN THE AIR, armed in HOLD. The same step tune, one wheel
-- at a time (forward then reverse), the other wheels at 0 A. Run 5 before 4:
-- 4 overwrites with the loaded vehicle.
-- SCR_USER1 = 6 is retired.

local K = {
  -- Shown in "ESC: ready" so a test can confirm which file is on the SD card
  VERSION = "v2611f",
  NUM_ESC = 4,
  MAX_FB = 8,
  CMD_ID = 0x200,
  FB_BASE = 0x200,
  COUNTS_PER_REV = 8192,
  RAW_PER_AMP = 16384 / 20,
  RAW_MAX = 16384,
  FEEDBACK_TIMEOUT_MS = 100,
  LOOP_MS = 5,
  -- Gains do not move at the control rate. Re-read them on this period.
  GAINS_MS = 50,
  PREARM_MS = 100,
  -- Read cap per cycle: the 25-frame buffer plus what arrives while reading.
  MAX_FRAMES = 40,
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
  -- Symmetrical optimum, h = 7. High plateau falls back to this below 60 rpm.
  TUNE_H = 7,
  TUNE_HI_RPM = 120,
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
  GCS_NAMES = {
    { "ESC1_I", "ESC1_ICMD", "ESC1_RPM" },
    { "ESC2_I", "ESC2_ICMD", "ESC2_RPM" },
    { "ESC3_I", "ESC3_ICMD", "ESC3_RPM" },
    { "ESC4_I", "ESC4_ICMD", "ESC4_RPM" },
  },
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
  { 20, "WT_RELAY", 100 },
  { 21, "WT_RPM", 120 },
  { 22, "CMD_L", 0 },
  { 23, "CMD_R", 0 },
  { 24, "CMD_SEQ", 0 },
  { 25, "STAT", 0 },
  { 26, "LOC_I", 0.5 },
  { 27, "FR1", 0 },
  { 28, "FR2", 0 },
  { 29, "FR3", 0 },
  { 30, "FR4", 0 },
  { 31, "P1", 0 },
  { 32, "I1", 0 },
  { 33, "FF1", 0 },
  { 34, "P2", 0 },
  { 35, "I2", 0 },
  { 36, "FF2", 0 },
  { 37, "P3", 0 },
  { 38, "I3", 0 },
  { 39, "FF3", 0 },
  { 40, "P4", 0 },
  { 41, "I4", 0 },
  { 42, "FF4", 0 },
  { 43, "GCS_HZ", 4 },
  { 44, "PREF", 0.5 },
  { 45, "WT_BW", 8 },
}

local MAV_SEVERITY = { CRITICAL = 2, ERROR = 3, WARNING = 4, NOTICE = 5, INFO = 6 }

-- Create ESC_* before require() and before the control loop. A missing module
-- or a later fault must not hide the parameter table. Key 74 stays; 72 is only
-- used when 74 is already stored under another prefix.
local table_ok = param:add_table(K.PARAM_KEY, K.PREFIX, #PARAM_SPEC)
if not table_ok then
  gcs:send_text(MAV_SEVERITY.WARNING, "ESC: key 74 blocked, trying 72")
  K.PARAM_KEY = 72
  table_ok = param:add_table(K.PARAM_KEY, K.PREFIX, #PARAM_SPEC)
end
if table_ok then
  for n = 1, #PARAM_SPEC do
    local item = PARAM_SPEC[n]
    if not param:add_param(K.PARAM_KEY, item[1], item[2], item[3]) then
      gcs:send_text(MAV_SEVERITY.ERROR, "ESC: failed to add " .. K.PREFIX .. item[2])
      table_ok = false
      break
    end
  end
end
if table_ok then
  gcs:send_text(MAV_SEVERITY.NOTICE, "ESC: parameters created")
else
  gcs:send_text(MAV_SEVERITY.CRITICAL, "ESC: could not create ESC_ parameters")
end


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
    integrator = 0,
    last_error = 0,
    ref_rpm = 0,
    ref_f = 0,
    target_rpm = 0,
    saturated = false,
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
  gains_ms = 0,
  tx = nil,
  telem = nil,
  latest = {},
  gcs_ms = 0,
  c620_seen = false,
  c620_ms = 0,
  prearm_ms = 0,
  -- Reused every control cycle so the 5 ms loop does not allocate.
  gains = {
    gear = 1,
    max_rpm = 1,
    max_current = 0,
    imax = 0,
    kp = 0,
    ki = 0,
    kd = 0,
    kff = 0,
    kp_w = { 0, 0, 0, 0 },
    ki_w = { 0, 0, 0, 0 },
    kff_w = { 0, 0, 0, 0 },
    speed_mode = true,
    opt = 0,
    brake = false,
    brake_current = 0,
    slip = 0,
    acc = 0,
    lmask = 0,
    gcs_hz = 0,
    pref = 0,
  },
  scratch = {
    sum = { 0, 0 },
    count = { 0, 0 },
    lag = {},
    fast = {},
    raw = { 0, 0, 0, 0, 0, 0, 0, 0 },
    goals = { 0, 0 },
  },
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

function M.fmt4(tag, t)
  return string.format("%s %.4f %.4f %.4f %.4f", tag, t[1] or 0, t[2] or 0, t[3] or 0, t[4] or 0)
end

-- dirs[direction][wheel] = { P, I, ff }. A wheel present in one direction and missing
-- in the other is a failed run. Wheels missing from every direction are skipped.
function M.save_wheel_gains(dirs)
  local p_acc, i_acc, f_acc, n = 0, 0, 0, 0
  local ps, is, fs = {}, {}, {}
  for w = 1, K.NUM_ESC do
    local p_sum, i_sum, f_sum, m = 0, 0, 0, 0
    for d = 1, #dirs do
      local r = dirs[d][w]
      if r ~= nil then
        if not finite(r.P) or not finite(r.I) or r.P <= 0 or r.I <= 0 or not finite(r.ff) or r.ff < 0 then
          return false, "invalid gains"
        end
        p_sum, i_sum, f_sum, m = p_sum + r.P, i_sum + r.I, f_sum + r.ff, m + 1
      end
    end
    if m > 0 and m ~= #dirs then
      return false, string.format("ESC %d missed a direction", w)
    end
    if m > 0 then
      local kp, ki, kff = p_sum / m, i_sum / m, f_sum / m
      M.queue_save(P["P" .. w], kp)
      M.queue_save(P["I" .. w], ki)
      M.queue_save(P["FF" .. w], kff)
      ps[w], is[w], fs[w] = kp, ki, kff
      p_acc, i_acc, f_acc, n = p_acc + kp, i_acc + ki, f_acc + kff, n + 1
    end
  end
  if n == 0 then
    return false, "no wheel tuned"
  end
  M.queue_save(P.P, p_acc / n)
  M.queue_save(P.I, i_acc / n)
  M.queue_save(P.FF, f_acc / n)
  M.queue_save(P.MODE, 0)
  return true, ps, is, fs
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
    local now = now_ms()
    if now - last_driver_warn >= 5000 then
      last_driver_warn = now
      gcs:send_text(MAV_SEVERITY.WARNING, "ESC: reboot so ESC_P1..FF4 can be added")
    end
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
  local wt_rpm = g("WT_RPM")
  local plateau = wt_rpm >= 60 and wt_rpm or K.TUNE_HI_RPM
  gcs:send_text(MAV_SEVERITY.NOTICE, string.format(
    "ESC: ready %s CAN=%.0f MODE=%.0f CUR=%.1f WT_RELAY=%.0f plateau=%.0f (WT_RPM=%.0f)",
    K.VERSION, g("CAN"), g("MODE"), g("CUR"), g("WT_RELAY"), plateau, wt_rpm))
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
  local lmask = M.gains.lmask & 0xFF
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

local function healthy(idx, now)
  local fb = esc[idx + 1]
  if not fb.detected or fb.last_ms == nil then
    return false
  end
  return (now or now_ms()) - fb.last_ms < K.FEEDBACK_TIMEOUT_MS
end

local function all_healthy()
  local saw = false
  local now = now_ms()
  for i = 0, K.MAX_FB - 1 do
    if is_throttle(i) or esc[i + 1].detected then
      saw = true
      if not healthy(i, now) then
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
    local fb = esc[i]
    fb.current_a = 0
    fb.integrator = 0
    fb.last_error = 0
    fb.ref_rpm = 0
    fb.ref_f = 0
    fb.target_rpm = 0
    fb.saturated = false
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
  M.gains_ms = 0
  M.prearm_ms = 0
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

-- frame is the newest valid feedback of ESC idx + 1 seen this cycle
local function accept_frame(frame, idx, now)
  local fb = esc[idx + 1]
  fb.angle = (((frame:data(0) & 0xFF) << 8) | (frame:data(1) & 0xFF)) % K.COUNTS_PER_REV
  fb.have_angle = true
  fb.rotor_rpm = be_i16(frame:data(2), frame:data(3))
  -- Frames are skipped, so the angle may wrap several times between two of
  -- them. Integrate the reported speed instead (used only by the DJIC log).
  if fb.last_ms ~= nil then
    fb.rotor_counts = fb.rotor_counts + fb.rotor_rpm * (now - fb.last_ms) * (K.COUNTS_PER_REV / 60000)
  end
  fb.current_raw = be_i16(frame:data(4), frame:data(5))
  fb.temperature = frame:data(6) & 0xFF
  fb.last_ms = now
  if not fb.detected then
    fb.detected = true
    gcs:send_text(MAV_SEVERITY.INFO, string.format("ESC: found ESC %d (%s)", idx + 1,
      side_of(idx) == 1 and "left" or "right"))
  end
end

--[[
Four C620s send about 4000 frames/s into a 25-frame buffer that drops the
newest frame when full. Reading only part of it left the oldest frames for
the next cycle, so the speed loop ran on feedback one cycle or more old.
Empty the buffer and decode only the newest frame of each ESC: the loop sees
current data and the VM does not parse frames it would throw away.
--]]
local function drain()
  local latest = M.latest
  for i = 1, K.MAX_FB do
    latest[i] = nil
  end
  for _ = 1, K.MAX_FRAMES do
    local frame = driver:read_frame()
    if not frame then
      break
    end
    -- No 0x7FF mask: an extended ID (29 bit) stays out of 1..8 instead of
    -- overwriting this cycle's C620 frame with the same low bits
    local n = frame:id_signed() - K.FB_BASE
    if n >= 1 and n <= K.MAX_FB then
      latest[n] = frame
      diag.n[n] = diag.n[n] + 1
    else
      diag.other = diag.other + 1
      diag.other_id = n + K.FB_BASE
    end
  end
  local now = now_ms()
  for n = 1, K.MAX_FB do
    local frame = latest[n]
    if frame ~= nil and not frame:isExtended() and not frame:isRemoteTransmissionRequest() and
        not frame:isErrorFrame() and frame:dlc() == 8 then
      accept_frame(frame, n - 1, now)
    end
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
  local frame = M.tx
  if frame == nil then
    frame = CANFrame()
    M.tx = frame
  end
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

local function profile_wheel(i, target_rpm, dt)
  local fb = esc[i + 1]
  local acc = M.gains.acc
  if acc > 0 then
    local step = acc * dt
    fb.ref_rpm = fb.ref_rpm + constrain(target_rpm - fb.ref_rpm, -step, step)
  else
    fb.ref_rpm = target_rpm
  end
end

-- Current window for a wheel spinning at rpm toward target.
-- Opposing current stays capped only when the reference still asks for this
-- direction and the wheel is already at least that fast. Slowing, stopping
-- or reversing opens the full ESC_CUR range, the same as a reversed PWM pulse.
local function current_window(rpm, target, k)
  local lo, hi = -k.max_current, k.max_current
  if not k.brake then
    return lo, hi
  end
  local brake = k.brake_current or 0
  if rpm > K.MOVING_RPM and target >= rpm then
    lo = -brake
  elseif rpm < -K.MOVING_RPM and target <= rpm then
    hi = brake
  end
  return lo, hi
end

-- 0 on a per-wheel gain means "use the shared ESC_P / ESC_I / ESC_FF".
local function wheel_pi(i, target, dt, k)
  local fb = esc[i + 1]
  fb.target_rpm = target
  local n = i + 1
  local kp = k.kp_w[n]
  local ki = k.ki_w[n]
  local kff = k.kff_w[n]
  if kp <= 0 then
    kp = k.kp
  end
  if ki <= 0 then
    ki = k.ki
  end
  if kff <= 0 then
    kff = k.kff
  end
  local err = target - fb.wheel_rpm
  local derivative = 0
  if dt > 0 then
    derivative = k.kd * (err - fb.last_error) / dt
  end
  fb.last_error = err
  local lo, hi = current_window(fb.wheel_rpm, target, k)
  local ff = kff * target
  local unsat = ff + kp * err + fb.integrator + derivative
  local saturated = (unsat >= hi and err > 0) or (unsat <= lo and err < 0)
  if not saturated then
    fb.integrator = constrain(fb.integrator + ki * err * dt, -k.imax, k.imax)
  end
  fb.saturated = (unsat > k.max_current and err > 0) or (unsat < -k.max_current and err < 0)
  return constrain(ff + kp * err + fb.integrator + derivative, lo, hi)
end

-- Integral time Ti = Kp / Ki of one wheel, with the shared-gain fallback of wheel_pi
local function wheel_ti(i, k)
  local kp = k.kp_w[i + 1]
  local ki = k.ki_w[i + 1]
  if kp <= 0 then
    kp = k.kp
  end
  if ki <= 0 then
    ki = k.ki
  end
  if kp <= 0 or ki <= 0 then
    return 0
  end
  return kp / ki
end

--[[
Reference prefilter: 1 / (1 + Ti s) on the speed reference cancels the PI zero
that makes the symmetrical-optimum loop overshoot a step by about 40%.
Only speeding up is filtered. Slowing down and stopping are passed at once,
and a change of direction restarts the filter from 0.
--]]
local function prefilter(fb, ref, dt, tf)
  if ref * fb.ref_f < 0 then
    fb.ref_f = 0
  end
  if tf > 0 and math.abs(ref) > math.abs(fb.ref_f) then
    fb.ref_f = fb.ref_f + (ref - fb.ref_f) * math.min(1, dt / tf)
  else
    fb.ref_f = ref
  end
  return fb.ref_f
end

local function read_gains(now)
  local cached = M.gains
  if M.gains_ms ~= 0 and now - M.gains_ms < K.GAINS_MS then
    return cached
  end
  M.gains_ms = now
  local max_current = constrain(g("CUR"), 0, 20)
  cached.gear = math.max(g("GEAR"), 1)
  cached.max_rpm = math.max(g("RPM"), 1)
  cached.max_current = max_current
  cached.imax = constrain(g("IMAX"), 0, max_current)
  cached.kp = g("P")
  cached.ki = g("I")
  cached.kd = g("D")
  cached.kff = g("FF")
  cached.kp_w[1] = math.max(g("P1"), 0)
  cached.kp_w[2] = math.max(g("P2"), 0)
  cached.kp_w[3] = math.max(g("P3"), 0)
  cached.kp_w[4] = math.max(g("P4"), 0)
  cached.ki_w[1] = math.max(g("I1"), 0)
  cached.ki_w[2] = math.max(g("I2"), 0)
  cached.ki_w[3] = math.max(g("I3"), 0)
  cached.ki_w[4] = math.max(g("I4"), 0)
  cached.kff_w[1] = math.max(g("FF1"), 0)
  cached.kff_w[2] = math.max(g("FF2"), 0)
  cached.kff_w[3] = math.max(g("FF3"), 0)
  cached.kff_w[4] = math.max(g("FF4"), 0)
  cached.speed_mode = g("MODE") < 0.5
  cached.opt = math.floor(g("OPT"))
  cached.brake = g("BRK") >= 0.5
  cached.brake_current = max_current * constrain(g("BRKCUR"), 0, 100) / 100
  cached.slip = math.max(g("SLIP"), 0)
  cached.acc = g("ACC")
  cached.lmask = math.floor(g("LMASK"))
  cached.gcs_hz = constrain(g("GCS_HZ"), 0, 20)
  cached.pref = constrain(g("PREF"), 0, 2)
  return cached
end

local function rc_centered()
  local t1 = rc1_trim:get() or 1500
  local t3 = rc3_trim:get() or 1500
  local p1 = rc:get_pwm(1) or t1
  local p3 = rc:get_pwm(3) or t3
  return math.abs(p1 - t1) < K.TUNE_STICK_US and math.abs(p3 - t3) < K.TUNE_STICK_US
end

-- USER1 = 4 (ground) or 5 (in the air); a running tune stops if the selector changes
local function tune_wanted(armed)
  local sel = selector()
  if sel ~= K.WHEEL_TUNE_SELECTOR and sel ~= K.BRK_SELECTOR then
    return false
  end
  if tune.mode ~= nil and tune.mode ~= sel then
    return false
  end
  return armed and vehicle:get_mode() == K.HOLD_MODE
end

local function tune_fail(msg)
  tune.state = "fail"
  tune_message(MAV_SEVERITY.ERROR, "failed - " .. msg .. "; gains kept")
end

-- Wheel speed is inertia plus a short lag, the plant the type-II rule is written
-- for (symmetrical optimum, mid-band ratio h = 7, phase margin about 46°).
-- Yaw and Circle keep the module default, Tyreus-Luyben.
-- Methods of M, not locals: Rover 4.5 allows 100 locals in the main chunk.
--[[
Heap note: on this board a table that grows (t[#t + 1] = v) is reallocated
only inside the first part of the script heap, which the loaded code fills.
Copies are made with {table.unpack(...)}, a single exact-size allocation that
may come from any part of the heap, and the plant lists are fixed size.
--]]
function M.sort_nums(t)
  local s = { table.unpack(t) }
  for i = 2, #s do
    local v = s[i]
    local j = i - 1
    while j >= 1 and s[j] > v do
      s[j + 1] = s[j]
      j = j - 1
    end
    s[j + 1] = v
  end
  return s
end

function M.median(t)
  local n = #t
  if n == 0 then
    return nil
  end
  local s = M.sort_nums(t)
  if n % 2 == 1 then
    return s[math.floor((n + 1) / 2)]
  end
  return 0.5 * (s[math.floor(n / 2)] + s[math.floor(n / 2) + 1])
end

-- Fixed 24-slot list: allocated once at its final size, never grown
K.PLANT_CAP = 24
function M.fixed_list()
  return { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
end

-- The first n entries of a fixed list, as an exact-size copy
function M.head(t, n)
  return { table.unpack(t, 1, n) }
end

function M.gains_sane(kp, ki, k)
  if kp == nil or ki == nil or not finite(kp) or not finite(ki) then
    return false
  end
  return kp > 0 and ki > 0 and kp < k.max_current and ki < k.max_current * 5
end

-- Current per rpm from two holds. A negative slope falls back to the high point.
function M.ff_slope(i_lo, rpm_lo, i_hi, rpm_hi)
  if i_lo ~= nil and rpm_lo ~= nil then
    local dw = math.abs(rpm_hi) - math.abs(rpm_lo)
    local di = math.abs(i_hi) - math.abs(i_lo)
    if dw > 1 and di >= 0 then
      return di / dw
    end
  end
  if math.abs(rpm_hi) >= 1 then
    return math.abs(i_hi) / math.abs(rpm_hi)
  end
  return nil
end

-- Pulse amplitude is a fraction of ESC_CUR, up to the whole current limit.
function M.relay_amp(k)
  local frac = constrain(g("WT_RELAY"), 1, 100) / 100
  return math.max(k.max_current * frac, 0.5)
end

function M.zero_currents()
  return { 0, 0, 0, 0, 0, 0, 0, 0 }
end

-- High plateau is ESC_WT_RPM. Below 60 rpm (the old saved default was 30) keep 120.
-- The low plateau is half of that. ESC_RPM caps both.
function M.tune_levels(k)
  local hi = g("WT_RPM")
  if hi < 60 then
    hi = K.TUNE_HI_RPM
  end
  if hi > k.max_rpm then
    hi = k.max_rpm
  end
  return hi * 0.5, hi
end

function M.tune_mag(k, level)
  local lo, hi = M.tune_levels(k)
  if level == 1 and lo ~= nil then
    return lo, true
  end
  return hi, lo ~= nil
end

function M.clear_wheel_pi()
  for i = 1, K.MAX_FB do
    esc[i].integrator = 0
    esc[i].last_error = 0
    esc[i].ref_rpm = 0
  end
end

function M.announce_saved(ps, is, fs)
  tune_message(MAV_SEVERITY.NOTICE, M.fmt4("P", ps))
  tune_message(MAV_SEVERITY.NOTICE, M.fmt4("I", is))
  tune_message(MAV_SEVERITY.NOTICE, M.fmt4("FF", fs))
  tune_message(MAV_SEVERITY.NOTICE, "SCR_USER1 set to 0 (normal)")
end

function M.tune_reset_window(zero_ref)
  tune.t = 0
  tune.count = 0
  tune.sum_current = { 0, 0, 0, 0 }
  tune.sum_rpm = { 0, 0, 0, 0 }
  if zero_ref then
    M.clear_wheel_pi()
  else
    for i = 1, K.MAX_FB do
      esc[i].integrator = 0
      esc[i].last_error = 0
    end
  end
end

function M.tune_start_direction(dir)
  tune.dir = dir
  tune.state = "ff"
  tune.ff_level = 1
  tune.lo_i = nil
  tune.lo_rpm = nil
  tune.base = { 0, 0, 0, 0 }
  tune.rpm = { 0, 0, 0, 0 }
  tune.relay = nil
  tune.plant = nil
  tune.plants = nil
  M.tune_reset_window(true)
end

--[[
Wheel tune constants. SCR_USER1 = 4 runs it with every wheel on the ground;
SCR_USER1 = 5 runs the same sequence for one wheel at a time, wheels in the
air, the others at 0 A.
Step test: from the high plateau, every wheel gets its hold current plus a
current step dI (ESC_WT_RELAY % of ESC_CUR), then minus dI. The rpm slope of
each wheel gives Kv (rpm/s per A) with the rover's own mass, and the line's
intercept with the starting rpm gives the lag T.
Gains: wc = min(ESC_WT_BW, 1 / (2.65 T)); Kp = wc / Kv; Ti = 4 / wc.
Verify: hold the low plateau, step the reference to the high plateau with the
new gains (and ESC_ACC / ESC_PREF as when driving), measure the overshoot.
Above 15%, wc shrinks by 25% and the check repeats (3 tries).
--]]
K.STEP_S = 0.25          -- longest current step
K.STEP_MOVE_RPM = 4      -- a wheel this far from its start rpm is moving: fit from here
K.STEP_MAX_DEV = 0.5     -- end the step early when a wheel is this fraction of the plateau away
K.STEP_MIN_N = 5         -- samples needed for a slope
K.RECOVER_S = 0.8
K.VERIFY_HOLD_S = 1.0
K.VERIFY_MEAS_S = 1.2
K.VERIFY_OS = 0.15
K.VERIFY_TRIES = 3
K.BW_SHRINK = 0.75
K.WC_LAG = 2.65          -- symmetrical optimum, h = 7: wc = 1 / (sqrt(h) T)
K.TI_WC = 4              -- Ti = 4 / wc

-- True for the wheels this run drives: all detected (USER1=4) or the one under test (USER1=5)
function M.on(w)
  return esc[w].detected and (tune.only == nil or tune.only == w)
end

-- Next detected wheel after w (nil when none), for the one-at-a-time run
function M.next_wheel(w)
  for i = w + 1, K.NUM_ESC do
    if esc[i].detected then
      return i
    end
  end
  return nil
end

function M.tune_finish()
  local ok, ps, is_, fs = M.save_wheel_gains(tune.rows)
  if not ok then
    tune_fail(ps)
    return
  end
  M.queue_save(scr_user1, 0)
  M.clear_wheel_pi()
  tune.state = "done"
  M.announce_saved(ps, is_, fs)
end

function M.tune_log_wheels(target, cur)
  if logger == nil then
    return
  end
  local ph = (tune.state == "step" and 2) or (tune.state == "verify" and 3) or 1
  for i = 1, K.NUM_ESC do
    if M.on(i) then
      logger:write("DJWT", "Side,Dir,Ph,Tgt,Rpm,Cur,Cyc", "BbBfffB",
        i, tune.dir or 0, ph, target, esc[i].wheel_rpm, cur[i], tune.tries or 0)
    end
  end
end

-- One current step: sign +1 speeds the wheels up (in the tune direction), -1 slows them
function M.step_begin(sign)
  local f = tune.fit
  tune.state = "step"
  tune.step_sign = sign
  tune.t = 0
  for w = 1, K.NUM_ESC do
    f.n[w], f.st[w], f.sr[w], f.stt[w], f.str[w] = 0, 0, 0, 0, 0
    f.r0[w] = tune.dir * esc[w].wheel_rpm
  end
end

-- Close one step: least-squares line rpm = a t + b on each wheel's moving samples
function M.step_close()
  local f = tune.fit
  local s = tune.step_sign
  for w = 1, K.NUM_ESC do
    local n = f.n[w]
    if M.on(w) and n >= K.STEP_MIN_N then
      local den = n * f.stt[w] - f.st[w] * f.st[w]
      if den > 0 then
        local a = (n * f.str[w] - f.st[w] * f.sr[w]) / den
        local b = (f.sr[w] - a * f.st[w]) / n
        local kv = s * a / tune.step_i
        local lag = (a ~= 0) and (f.r0[w] - b) / a or -1
        if finite(kv) and kv > 0 and finite(lag) and lag >= 0 and lag <= 0.3 then
          f.kv[w] = f.kv[w] + kv
          f.kvn[w] = f.kvn[w] + 1
          f.lag[w] = f.lag[w] + lag
        end
      end
    end
  end
end

-- Gains for every wheel from its Kv, the median lag and the current bandwidth scale
function M.step_gains(k)
  local f = tune.fit
  local lags = M.fixed_list()
  local nl = 0
  for w = 1, K.NUM_ESC do
    if M.on(w) then
      if f.kvn[w] == 0 then
        return false, string.format("ESC %d no step response", w)
      end
      nl = nl + 1
      lags[nl] = f.lag[w] / f.kvn[w]
    end
  end
  local lag = M.median(M.head(lags, nl))
  if lag == nil then
    return false, "no step response"
  end
  local bw = constrain(g("WT_BW"), 1, 50)
  if lag > 0 then
    bw = math.min(bw, 1 / (K.WC_LAG * lag))
  end
  tune.lag = lag
  tune.wc = bw * tune.bw_scale
  local ti = K.TI_WC / tune.wc
  for w = 1, K.NUM_ESC do
    if M.on(w) then
      local kv = f.kv[w] / f.kvn[w]
      local kp = tune.wc / kv
      local ki = kp / ti
      if not M.gains_sane(kp, ki, k) then
        return false, string.format("ESC %d gains out of range (Kv %.0f)", w, kv)
      end
      tune.tkp[w], tune.tki[w] = kp, ki
    end
  end
  return true
end

-- Start the check on the low plateau with each integrator already holding the
-- current measured there (minus its FF). From zero, a low-gain wheel stalls on
-- static friction, the integrator winds up and the wheel overshoots on breakaway.
function M.verify_begin(k)
  tune.state = "verify"
  tune.vphase = "hold"
  tune.t = 0
  for w = 1, K.NUM_ESC do
    tune.peak[w] = 0
    local fb = esc[w]
    fb.last_error = 0
    fb.integrator = 0
    if M.on(w) and tune.lo_i ~= nil then
      fb.integrator = constrain(tune.lo_i[w] - tune.tff[w] * tune.lo_rpm[w], -k.imax, k.imax)
    end
  end
  tune.imax = 0
end

function M.tune_start_direction(dir)
  tune.dir = dir
  tune.state = "ff"
  tune.ff_level = 1
  tune.lo_i = nil
  tune.lo_rpm = nil
  tune.base = { 0, 0, 0, 0 }
  tune.rpm = { 0, 0, 0, 0 }
  tune.bw_scale = 1
  tune.tries = 0
  -- Constructors: every list at its final size, no growth during the tune
  tune.fit = { n = { 0, 0, 0, 0 }, st = { 0, 0, 0, 0 }, sr = { 0, 0, 0, 0 }, stt = { 0, 0, 0, 0 },
    str = { 0, 0, 0, 0 }, r0 = { 0, 0, 0, 0 }, kv = { 0, 0, 0, 0 }, kvn = { 0, 0, 0, 0 },
    lag = { 0, 0, 0, 0 } }
  tune.tkp = { 0, 0, 0, 0 }
  tune.tki = { 0, 0, 0, 0 }
  tune.tff = { 0, 0, 0, 0 }
  tune.peak = { 0, 0, 0, 0 }
  -- Best check so far: shrinking the bandwidth does not always help (static
  -- friction with low gains), so the lowest overshoot is what gets saved
  tune.best_os = math.huge
  tune.bkp = { 0, 0, 0, 0 }
  tune.bki = { 0, 0, 0, 0 }
  tune.best_wc = 0
  tune.kt = nil
  M.tune_reset_window(true)
end

-- Store this direction's gains and move on: reverse, the next wheel (USER1=5) or save
function M.direction_done()
  local row = tune.rows[tune.dir > 0 and 1 or 2]
  for w = 1, K.NUM_ESC do
    if M.on(w) then
      row[w] = { P = tune.tkp[w], I = tune.tki[w], ff = tune.tff[w] }
    end
  end
  if tune.dir > 0 then
    M.tune_start_direction(-1)
  elseif tune.only ~= nil and M.next_wheel(tune.only) ~= nil then
    tune.only = M.next_wheel(tune.only)
    tune_message(MAV_SEVERITY.INFO, string.format("next ESC %d", tune.only))
    M.tune_start_direction(1)
    -- Let the previous wheel spin down before the next one starts
    tune.state = "rest"
  else
    M.tune_finish()
  end
end

--[[
Wheel tune (SCR_USER1 = 4 on the ground, 5 in the air one wheel at a time).
Returns per-wheel currents while it owns the outputs, or nil to let the
normal speed loop run.
"rest"    USER1=5 only: all wheels at 0 A between two wheels
"ff"      hold 60 rpm, then 120 rpm; current slope of each wheel = ESC_FFn
"step"    current step up then down at the high plateau: Kv and T of each wheel
"recover" back to the plateau with the speed loop
"verify"  low plateau, then a reference step to the high one with the new gains
Forward first, then reverse.
--]]
function M.wheel_tune(dt, k, armed, enabled)
  local wanted = tune_wanted(armed)
  if not wanted then
    if tune.state ~= "idle" and tune.state ~= "done" and tune.state ~= "fail" then
      tune_message(MAV_SEVERITY.WARNING, "aborted (left HOLD, disarmed or selector changed)")
    end
    if tune.state ~= "idle" then
      tune_reset()
      M.clear_wheel_pi()
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
    tune.mode = selector()
    tune.rows = { {}, {} }
    tune.only = nil
    if tune.mode == K.BRK_SELECTOR then
      tune.only = M.next_wheel(0)
      if tune.only == nil then
        tune_fail("no ESC detected")
        return nil
      end
    end
    M.tune_start_direction(1)
    local lo, hi = M.tune_levels(k)
    tune_message(MAV_SEVERITY.NOTICE, string.format("%s step tune %.0f/%.0f rpm, step %.1fA, BW %.1f",
      tune.only and "in the air, one wheel," or "ground", lo, hi, M.relay_amp(k), constrain(g("WT_BW"), 1, 50)))
  end
  if not enabled then
    tune_fail("ESC feedback lost or outputs disabled")
    return nil
  end
  if not rc_centered() then
    tune_fail("stick moved")
    return nil
  end

  local lo_rpm, hi_rpm = M.tune_levels(k)
  local dir = tune.dir
  local dir_name = dir > 0 and "fwd" or "rev"
  local cur = M.zero_currents()
  tune.t = tune.t + dt
  local target = dir * (tune.ff_level == 1 and lo_rpm or hi_rpm)

  if tune.state == "rest" then
    if tune.t >= K.BRK_REST_S then
      M.tune_start_direction(dir)
    end
    return cur
  elseif tune.state == "ff" then
    local any = false
    for i = 0, K.NUM_ESC - 1 do
      local fb = esc[i + 1]
      if M.on(i + 1) then
        any = true
        profile_wheel(i, target, dt)
        cur[i + 1] = wheel_pi(i, fb.ref_rpm, dt, k)
        if tune.t >= K.TUNE_SETTLE_S then
          tune.sum_current[i + 1] = tune.sum_current[i + 1] + cur[i + 1]
          tune.sum_rpm[i + 1] = tune.sum_rpm[i + 1] + fb.wheel_rpm
        end
      end
    end
    if not any then
      tune_fail("no ESC detected")
      return M.zero_currents()
    end
    if tune.t >= K.TUNE_SETTLE_S then
      tune.count = tune.count + 1
    end
    if tune.t >= K.TUNE_SETTLE_S + K.TUNE_AVERAGE_S then
      local means_i, means_r = { 0, 0, 0, 0 }, { 0, 0, 0, 0 }
      for i = 0, K.NUM_ESC - 1 do
        if M.on(i + 1) then
          local mean_rpm = tune.sum_rpm[i + 1] / tune.count
          if math.abs(mean_rpm - target) > 0.3 * math.abs(target) then
            tune_fail(string.format("ESC %d did not reach %.0f rpm (%.0f)", i + 1, target, mean_rpm))
            return M.zero_currents()
          end
          means_i[i + 1] = tune.sum_current[i + 1] / tune.count
          means_r[i + 1] = mean_rpm
          esc[i + 1].ref_rpm = target
        end
      end
      if tune.ff_level == 1 then
        tune.lo_i = means_i
        tune.lo_rpm = means_r
        tune.ff_level = 2
        M.tune_reset_window(false)
      else
        tune.base = means_i
        tune.rpm = means_r
        for w = 1, K.NUM_ESC do
          if M.on(w) then
            local ff = M.ff_slope(tune.lo_i[w], tune.lo_rpm[w], means_i[w], means_r[w])
            if ff == nil or not finite(ff) or ff < 0 then
              tune_fail(string.format("ESC %d rpm too low for FF", w))
              return M.zero_currents()
            end
            tune.tff[w] = ff
          end
        end
        tune.step_i = M.relay_amp(k)
        M.step_begin(1)
      end
    end
  elseif tune.state == "step" then
    local s = tune.step_sign
    local f = tune.fit
    local over = false
    for w = 1, K.NUM_ESC do
      local fb = esc[w]
      if M.on(w) then
        cur[w] = constrain(tune.base[w] + dir * s * tune.step_i, -k.max_current, k.max_current)
        local r = dir * fb.wheel_rpm
        if math.abs(r - f.r0[w]) >= K.STEP_MOVE_RPM then
          local t = tune.t
          f.n[w] = f.n[w] + 1
          f.st[w] = f.st[w] + t
          f.sr[w] = f.sr[w] + r
          f.stt[w] = f.stt[w] + t * t
          f.str[w] = f.str[w] + t * r
        end
        if math.abs(r - hi_rpm) > K.STEP_MAX_DEV * hi_rpm then
          over = true
        end
      end
    end
    if over or tune.t >= K.STEP_S then
      M.step_close()
      if s > 0 then
        M.step_begin(-1)
      else
        tune.state = "recover"
        tune.t = 0
        M.clear_wheel_pi()
      end
    end
  elseif tune.state == "recover" then
    for i = 0, K.NUM_ESC - 1 do
      if M.on(i + 1) then
        profile_wheel(i, target, dt)
        cur[i + 1] = wheel_pi(i, esc[i + 1].ref_rpm, dt, k)
      end
    end
    if tune.t >= K.RECOVER_S then
      local ok, why = M.step_gains(k)
      if not ok then
        tune_fail(why)
        return M.zero_currents()
      end
      local f = tune.fit
      local kvs = { 0, 0, 0, 0 }
      for w = 1, K.NUM_ESC do
        kvs[w] = f.kvn[w] > 0 and f.kv[w] / f.kvn[w] or 0
      end
      tune_message(MAV_SEVERITY.INFO, string.format("%s Kv %.0f %.0f %.0f %.0f T %.3f",
        dir_name, kvs[1], kvs[2], kvs[3], kvs[4], tune.lag))
      M.verify_begin(k)
    end
  elseif tune.state == "verify" then
    -- New gains through a proxy of the cached gains, with ESC_ACC and ESC_PREF as when driving
    local kt = tune.kt
    if kt == nil then
      kt = setmetatable({ kp_w = tune.tkp, ki_w = tune.tki, kff_w = tune.tff }, { __index = k })
      tune.kt = kt
    end
    local ref_goal = dir * (tune.vphase == "hold" and lo_rpm or hi_rpm)
    for i = 0, K.NUM_ESC - 1 do
      local fb = esc[i + 1]
      if M.on(i + 1) then
        profile_wheel(i, ref_goal, dt)
        local ref = fb.ref_rpm
        if k.pref > 0 then
          ref = prefilter(fb, ref, dt, (tune.tkp[i + 1] / tune.tki[i + 1]) * k.pref)
        end
        cur[i + 1] = wheel_pi(i, ref, dt, kt)
        if tune.vphase == "meas" then
          local r = dir * fb.wheel_rpm
          if r > tune.peak[i + 1] then
            tune.peak[i + 1] = r
          end
          if math.abs(cur[i + 1]) > tune.imax then
            tune.imax = math.abs(cur[i + 1])
          end
        end
      end
    end
    if tune.vphase == "hold" and tune.t >= K.VERIFY_HOLD_S then
      tune.vphase = "meas"
      tune.t = 0
    elseif tune.vphase == "meas" and tune.t >= K.VERIFY_MEAS_S then
      local os = 0
      for w = 1, K.NUM_ESC do
        if M.on(w) then
          os = math.max(os, (tune.peak[w] - hi_rpm) / (hi_rpm - lo_rpm))
        end
      end
      tune.tries = tune.tries + 1
      tune_message(MAV_SEVERITY.INFO, string.format("%s check %d: BW %.1f overshoot %.0f%% peak %.1fA",
        dir_name, tune.tries, tune.wc, math.max(os, 0) * 100, tune.imax))
      local improved = os < tune.best_os
      if improved then
        tune.best_os, tune.best_wc = os, tune.wc
        for w = 1, K.NUM_ESC do
          tune.bkp[w], tune.bki[w] = tune.tkp[w], tune.tki[w]
        end
      end
      if os > K.VERIFY_OS and tune.tries < K.VERIFY_TRIES and improved then
        tune.bw_scale = tune.bw_scale * K.BW_SHRINK
        local ok, why = M.step_gains(k)
        if not ok then
          tune_fail(why)
          return M.zero_currents()
        end
        M.verify_begin(k)
      else
        for w = 1, K.NUM_ESC do
          tune.tkp[w], tune.tki[w] = tune.bkp[w], tune.bki[w]
        end
        if tune.best_os > K.VERIFY_OS then
          tune_message(MAV_SEVERITY.WARNING, string.format("%s best overshoot %.0f%%; keeping BW %.1f",
            dir_name, tune.best_os * 100, tune.best_wc))
        end
        tune_message(MAV_SEVERITY.INFO, M.fmt4(dir_name .. " P", tune.tkp))
        tune_message(MAV_SEVERITY.INFO, M.fmt4(dir_name .. " I", tune.tki))
        M.direction_done()
        return M.zero_currents()
      end
    end
  end
  M.tune_log_wheels(target, cur)
  return cur
end

function M.check_slip(k, now, enabled)
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

function M.run_control()
  local now_us = micros()
  local dt = 0.01
  if last_us ~= nil then
    dt = constrain(as_number(now_us - last_us) * 1.0e-6, 0, 0.05)
  end
  last_us = now_us
  local now = now_ms()

  local k = read_gains(now)
  local hold_disarmed = (k.opt & 2) ~= 0
  local armed = arming:is_armed()
  local safety_live = not SRV_Channels:get_safety_state()
  local estop = SRV_Channels:get_emergency_stop()
  local outputs_ok = latched_can and safety_live and not estop and mask_valid() and all_healthy()
  local enabled = outputs_ok and (armed or (k.speed_mode and hold_disarmed))

  -- Feedback per wheel and per side.
  -- rpm_mean / rpm_lag feed the slip warning and desaturation. Each wheel PI
  -- closes on its own rpm, so a free-spinning wheel cannot satisfy the other.
  local scratch = M.scratch
  local sum = scratch.sum
  local count = scratch.count
  local lag = scratch.lag
  local fast = scratch.fast
  sum[1], sum[2] = 0, 0
  count[1], count[2] = 0, 0
  lag[1], lag[2] = nil, nil
  fast[1], fast[2] = nil, nil
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

  local wheel_currents = M.wheel_tune(dt, k, armed, enabled)
  if not enabled then
    wheel_currents = nil
  end

  local raw = scratch.raw
  for i = 1, K.MAX_FB do
    raw[i] = 0
  end
  if not enabled then
    reset_control()
  elseif wheel_currents ~= nil then
    side[1].saturated = false
    side[2].saturated = false
    for i = 0, K.MAX_FB - 1 do
      local fb = esc[i + 1]
      if fb.detected then
        fb.current_a = constrain(wheel_currents[i + 1], -k.max_current, k.max_current)
        raw[i + 1] = direction(i) * fb.current_a * K.RAW_PER_AMP
      end
    end
  elseif k.speed_mode then
    local goals = scratch.goals
    for s = 1, 2 do
      goals[s] = side[s].interp_value * k.max_rpm
    end
    -- A saturated side cannot reach its reference. Scale the other side by the
    -- fraction this side achieved so the turn radius stays put. Use the lagging
    -- wheel when the pair has split, otherwise the free wheel looks like success.
    if (k.opt & 4) ~= 0 then
      for s = 1, 2 do
        local o = 3 - s
        if side[s].saturated and not side[o].saturated and math.abs(goals[s]) > K.MOVING_RPM then
          local achieved = side[s].diverged and side[s].rpm_lag or side[s].rpm_mean
          goals[o] = goals[o] * constrain(achieved / goals[s], 0, 1)
        end
      end
    end
    for s = 1, 2 do
      side[s].saturated = false
      side[s].target_rpm = goals[s]
    end
    for i = 0, K.MAX_FB - 1 do
      local fb = esc[i + 1]
      if fb.detected then
        local s = side_of(i)
        local sd = side[s]
        profile_wheel(i, goals[s], dt)
        local at_rest = math.abs(sd.interp_value) < 0.01 and math.abs(fb.ref_rpm) < K.MOVING_RPM
            and math.abs(fb.wheel_rpm) < K.MOVING_RPM
        if at_rest then
          fb.integrator = 0
          fb.last_error = 0
          fb.ref_f = 0
        end
        if at_rest and not hold_disarmed then
          -- Neutral stick with the wheel stopped: zero current, like a PWM
          -- neutral pulse. A speed loop held at 0 rpm through the gearbox
          -- backlash and the rpm noise makes the motor buzz and shake.
          fb.target_rpm = 0
          fb.saturated = false
          fb.current_a = 0
        else
          local ref = fb.ref_rpm
          if k.pref > 0 then
            ref = prefilter(fb, ref, dt, wheel_ti(i, k) * k.pref)
          end
          fb.current_a = wheel_pi(i, ref, dt, k)
        end
        if fb.saturated then
          sd.saturated = true
        end
        sd.current = fb.current_a
        raw[i + 1] = direction(i) * fb.current_a * K.RAW_PER_AMP
      end
    end
    if side[1].saturated or side[2].saturated then
      sat_ms = now
    end
  else
    for s = 1, 2 do
      side[s].target_rpm = 0
      side[s].saturated = false
    end
    for i = 0, K.MAX_FB - 1 do
      local fb = esc[i + 1]
      if fb.detected then
        local sd = side[side_of(i)]
        fb.integrator = 0
        fb.last_error = 0
        fb.target_rpm = 0
        fb.saturated = false
        fb.current_a = constrain(sd.interp_value * k.max_current, -k.max_current, k.max_current)
        sd.current = fb.current_a
        raw[i + 1] = direction(i) * fb.current_a * K.RAW_PER_AMP
      end
    end
  end
  M.check_slip(k, now, enabled)

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

function M.check_health()
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
        local fb = esc[i + 1]
        gcs:send_text(MAV_SEVERITY.CRITICAL, string.format("ESC: ESC %d lost %.0fms (%.1fA %dC), motors stopped",
          i + 1, now - (fb.last_ms or now), math.abs(fb.current_raw) / K.RAW_PER_AMP, fb.temperature))
      end
    end
  end
  if lost == 0 and lost_mask ~= 0 then
    gcs:send_text(MAV_SEVERITY.INFO, "ESC: feedback restored")
  end
  lost_mask = lost
end

function M.prearm()
  if auth_id == nil then
    return
  end
  local now = now_ms()
  if now - M.prearm_ms < K.PREARM_MS then
    return
  end
  M.prearm_ms = now
  if not latched_can then
    arming:set_aux_auth_passed(auth_id)
    return
  end
  -- Name lookup walks the whole parameter tree. The old script does not appear mid-flight.
  if now - M.c620_ms >= 1000 then
    M.c620_ms = now
    M.c620_seen = param:get('C620_CAN') ~= nil
  end
  if M.c620_seen then
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

function M.publish_stat()
  local stat = 0
  if latched_can and driver ~= nil then
    stat = stat | K.STAT_CAN
  end
  if sat_ms ~= nil and now_ms() - sat_ms < K.SAT_HOLD_MS then
    stat = stat | K.STAT_SAT
  end
  if tune.state == "ff" or tune.state == "step" or tune.state == "recover" or
    tune.state == "verify" or tune.state == "rest" then
    stat = stat | K.STAT_TUNE_RUN
  elseif tune.state == "done" then
    stat = stat | K.STAT_TUNE_OK
  elseif tune.state == "fail" then
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

function M.write_log()
  if (M.gains.opt & 1) == 0 or logger == nil then
    return
  end
  local now = now_ms()
  if now - last_log < 50 then
    return
  end
  last_log = now
  local gear = math.max(M.gains.gear, 1)
  for i = 0, K.NUM_ESC - 1 do
    local fb = esc[i + 1]
    if fb.detected then
      local dir = direction(i)
      logger:write("DJIC", "Id,Tgt,Rpm,CurD,CurM,I,Pos,Temp", "BffffffB",
        i + 1,
        fb.target_rpm,
        fb.wheel_rpm,
        fb.current_a,
        dir * fb.current_raw / K.RAW_PER_AMP,
        fb.integrator,
        dir * fb.rotor_counts / (K.COUNTS_PER_REV * gear),
        fb.temperature)
    end
  end
end

function M.push_telem()
  local data = M.telem
  for i = 0, K.NUM_ESC - 1 do
    local fb = esc[i + 1]
    if fb.detected and healthy(i) then
      data:motor_temp_cdeg(math.floor(constrain(fb.temperature, 0, 200) * 100))
      data:current(math.abs(fb.current_raw) / K.RAW_PER_AMP)
      esc_telem:update_telem_data(i, data, K.TELEM_MOTOR_TEMP | K.TELEM_CURRENT)
      esc_telem:update_rpm(i, math.abs(fb.rotor_rpm), 0)
    end
  end
end

-- Live per-motor values for the GCS (Mission Planner: Quick tab or Tuning graph)
function M.send_gcs()
  local hz = M.gains.gcs_hz
  if hz <= 0 or not latched_can then
    return
  end
  local now = now_ms()
  if now - M.gcs_ms < 1000 / hz then
    return
  end
  M.gcs_ms = now
  for i = 1, K.NUM_ESC do
    local fb = esc[i]
    if fb.detected then
      -- Constant names: no new strings on every send
      local names = K.GCS_NAMES[i]
      gcs:send_named_float(names[1], direction(i - 1) * fb.current_raw / K.RAW_PER_AMP)
      gcs:send_named_float(names[2], fb.current_a)
      gcs:send_named_float(names[3], fb.wheel_rpm)
    end
  end
  gcs:send_named_float("ESC_TGT_L", side[1].target_rpm or 0)
  gcs:send_named_float("ESC_TGT_R", side[2].target_rpm or 0)
end

function M.send_telem()
  if esc_telem == nil or now_ms() - last_telem < 20 then
    return
  end
  last_telem = now_ms()
  if M.telem == nil then
    local ok, data = pcall(ESCTelemetryData)
    if not ok or data == nil then
      return
    end
    M.telem = data
  end
  pcall(M.push_telem)
end

-- While a motor ESC is missing, report the frames seen per ID every 5 s so a
-- silent ESC (PWM mode, wrong ID, wiring) can be told apart from a slow loop
function M.report_rx()
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

function M.warn_selector_without_can()
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
      M.publish_stat()
      return update, 1000
    end
    -- Keep only 0x200..0x20F (C620 feedback) in the script buffer so other
    -- traffic on the bus cannot push ESC frames out of it.
    driver:add_filter(0x7F0, 0x200)
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
  -- Fill the gain cache before mask_valid() and the control loop read it.
  read_gains(now_ms())
  apply_switch()
  if latched_can and not mask_valid() then
    local now = now_ms()
    if now - last_mask_warn >= 5000 then
      last_mask_warn = now
      gcs:send_text(MAV_SEVERITY.ERROR, "ESC: ESC_LMASK must put ESCs on both sides")
    end
  end
  -- Zero MAIN 1..4 every cycle. The override lasts 3 s, so a second call in
  -- the same cycle only cost VM time.
  if latched_can then
    kill_pwm()
  end
  drain()
  M.run_control()
  if latched_can then
    M.check_health()
  end
  M.warn_selector_without_can()
  M.report_rx()
  M.publish_stat()
  M.prearm()
  -- Spread EEPROM / telem / log so a single update stays under SCR_VM_I_COUNT
  M.drain_saves()
  local tick = now_ms() % 100
  if tick < 50 then
    M.send_telem()
  else
    M.write_log()
  end
  M.send_gcs()
  return update, K.LOOP_MS
end

--[[
A runtime error must not end the script: once it stops, the kill_pwm override
expires after 3 s and Rover's own servo output reaches the C620 PWM inputs.
After the first fault the driver latches a safe stop until reboot: zero
current on the bus, MAIN 1..4 held at width 0 and arming refused.
--]]
M.fault = nil
M.fault_warn_ms = 0

function M.safe_stop()
  if latched_can then
    kill_pwm()
    if driver ~= nil then
      local zeros = M.zero_currents()
      send_group(0x200, zeros, 1)
      for i = 5, K.MAX_FB do
        if esc[i].detected then
          send_group(0x1FF, zeros, 5)
          break
        end
      end
    end
  end
  if auth_id ~= nil then
    arming:set_aux_auth_failed(auth_id, "ESC script fault; reboot")
  end
end

local function protected_update()
  if M.fault == nil then
    local ok, result, delay = pcall(update)
    if ok then
      return protected_update, delay
    end
    M.fault = tostring(result)
    M.fault_warn_ms = 0
  end
  pcall(M.safe_stop)
  local now = now_ms()
  if now - M.fault_warn_ms >= 5000 then
    M.fault_warn_ms = now
    gcs:send_text(MAV_SEVERITY.CRITICAL, "ESC: fault, motors stopped; reboot")
    gcs:send_text(MAV_SEVERITY.CRITICAL, "ESC: " .. string.sub(M.fault, 1, 40))
  end
  return protected_update, 100
end

return protected_update, 2000
