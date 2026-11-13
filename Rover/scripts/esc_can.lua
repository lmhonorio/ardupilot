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
-- ESC_WT_RELAY wheel autotune relay amplitude, % of ESC_CUR. Default 100,
--              so the pulse runs at the current limit. 1..100, no extra amp cap.
-- ESC_WT_RPM   kept so the parameter table does not shift. The test speed is
--              fixed: 120 rpm high, 60 rpm low.
--
-- Both wheels of a side share the side command as their speed target. Each closes
-- its own PI on its own encoder, so a slipping wheel cannot satisfy the loop of
-- the wheel that is still on the ground.
--
-- SCR_USER1 = 4, armed in HOLD, on the ground. Synchronised relay: every wheel
-- gets the same extra current, switched on the mean speed error, so the rover
-- drives straight. One PI (type II, h = 7) from the mean plant goes to every
-- wheel. ESC_FFn is the current slope of that wheel between two speeds.
-- SCR_USER1 = 5, WHEELS IN THE AIR, armed in HOLD. One wheel at a time, forward
-- then reverse, other wheels at 0 A. Same rule, per motor. Run 5 before 4:
-- 4 overwrites with the loaded vehicle.
-- SCR_USER1 = 6 is retired.

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
  LOOP_MS = 5,
  -- Gains do not move at the control rate. Re-read them on this period.
  GAINS_MS = 50,
  PREARM_MS = 100,
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
  { 20, "WT_RELAY", 100 },
  { 21, "WT_RPM", 30 },
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
    local fb = esc[i]
    fb.current_a = 0
    fb.integrator = 0
    fb.last_error = 0
    fb.ref_rpm = 0
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
  local lo, hi = current_window(fb.wheel_rpm, k)
  local ff = kff * target
  local unsat = ff + kp * err + fb.integrator + derivative
  local saturated = (unsat >= hi and err > 0) or (unsat <= lo and err < 0)
  if not saturated then
    fb.integrator = constrain(fb.integrator + ki * err * dt, -k.imax, k.imax)
  end
  fb.saturated = (unsat > k.max_current and err > 0) or (unsat < -k.max_current and err < 0)
  return constrain(ff + kp * err + fb.integrator + derivative, lo, hi)
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
  return cached
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

-- Wheel speed is inertia plus a short lag, the plant the type-II rule is written
-- for (symmetrical optimum, mid-band ratio h = 7, phase margin about 46°).
-- Yaw and Circle keep the module default, Tyreus-Luyben.
local TUNE_H = 7

local function sort_nums(t)
  local s = {}
  for i = 1, #t do
    s[i] = t[i]
  end
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

local function median(t)
  local n = #t
  if n == 0 then
    return nil
  end
  local s = sort_nums(t)
  if n % 2 == 1 then
    return s[math.floor((n + 1) / 2)]
  end
  return 0.5 * (s[math.floor(n / 2)] + s[math.floor(n / 2) + 1])
end

local function rel_spread(t, m)
  if m == nil or m <= 0 then
    return math.huge
  end
  local d = 0
  for _, v in ipairs(t) do
    d = math.max(d, math.abs(v - m) / m)
  end
  return d
end

-- nil when there are too few samples or they disagree. min_n keeps a single bump out.
local function robust_median(t, max_spread, min_n)
  if #t < min_n then
    return nil, "short"
  end
  local m = median(t)
  if m == nil or m <= 0 then
    return nil, "short"
  end
  if rel_spread(t, m) > max_spread then
    return nil, "spread"
  end
  return m, nil
end

local function gains_sane(kp, ki, k)
  if kp == nil or ki == nil or not finite(kp) or not finite(ki) then
    return false
  end
  return kp > 0 and ki > 0 and kp < k.max_current and ki < k.max_current * 5
end

-- Kp in A/rpm, Kv in rpm/s per A, T in s. Ti = h*T, Kp = (h+1)/(2*h*T*Kv).
local function gains_type2(kv, lag)
  if kv == nil or lag == nil or kv <= 0 or lag <= 0 then
    return nil, nil
  end
  local ti = TUNE_H * lag
  local kp = (TUNE_H + 1) / (2 * TUNE_H * lag * kv)
  if ti <= 0 or kp <= 0 then
    return nil, nil
  end
  return kp, kp / ti
end

-- h = 7 mapped onto a true -180° point: Kp = 0.36 Ku, Ti = 1.75 Pu.
local function gains_fallback(ku, pu)
  if ku == nil or pu == nil or ku <= 0 or pu <= 0 then
    return nil, nil
  end
  local kp = 0.36 * ku
  local ti = 1.75 * pu
  return kp, kp / ti
end

local function new_fourier()
  return { sum_s = 0, sum_c = 0, sum_t = 0, phase = 0, period = nil, amps = {} }
end

local function fourier_arm(f, period)
  f.sum_s, f.sum_c, f.sum_t, f.phase = 0, 0, 0, 0
  f.period = period
end

local function fourier_step(f, err, dt)
  if f.period == nil or f.period <= 0 then
    return
  end
  f.phase = f.phase + (2 * math.pi * dt / f.period)
  f.sum_s = f.sum_s + err * math.sin(f.phase) * dt
  f.sum_c = f.sum_c + err * math.cos(f.phase) * dt
  f.sum_t = f.sum_t + dt
end

local function fourier_close(f)
  if f.period == nil or f.sum_t < 0.5 * f.period then
    return
  end
  local a = 2 * math.sqrt(f.sum_s * f.sum_s + f.sum_c * f.sum_c) / f.sum_t
  if a > 0 then
    f.amps[#f.amps + 1] = a
  end
end

-- Close the window that just finished, then arm the next one at the measured period.
local function track_cycle(relay, fou, err, dt, prev_cycle)
  fourier_step(fou, err, dt)
  if relay.cycle == prev_cycle then
    return prev_cycle
  end
  if relay.cycle > relay.skip_cycles then
    fourier_close(fou)
  end
  if relay.last_period ~= nil and relay.last_period > 0 then
    fourier_arm(fou, relay.last_period)
  end
  return relay.cycle
end

-- Kv from the rpm slope just after the lag, T from the switch-to-peak delay.
local function new_plant(turn_rpm)
  return {
    t = 0,
    prev_u = nil,
    prev_rpm = nil,
    nswitch = 0,
    kvs = {},
    ts = {},
    turn_rpm = turn_rpm,
  }
end

local function plant_close(p)
  local u = p.u_hold
  if u ~= nil and u ~= 0 and p.ramp_t ~= nil then
    local rpm1, t1 = p.kv_rpm, p.kv_t
    if rpm1 == nil then
      rpm1, t1 = p.prev_rpm, p.t
    end
    local dur = t1 - p.ramp_t
    if dur >= 0.03 and rpm1 ~= nil and math.abs(rpm1 - p.ramp_rpm) >= 1 then
      local kv = ((rpm1 - p.ramp_rpm) / dur) / u
      if kv > 0 and kv < 1e6 then
        p.kvs[#p.kvs + 1] = kv
      end
    end
  end
  if p.turned and p.peak_t ~= nil and p.switch_t ~= nil then
    local lag = p.peak_t - p.switch_t
    if lag >= 0.01 and lag <= 0.8 then
      p.ts[#p.ts + 1] = lag
    end
  end
end

local function plant_begin(p, rpm, u)
  p.switch_t = p.t
  p.switch_rpm = rpm
  p.peak_rpm = rpm
  p.peak_t = p.t
  p.turned = false
  p.ramp_t = nil
  p.ramp_rpm = nil
  p.kv_rpm = nil
  p.kv_t = nil
  p.u_hold = u
end

local function plant_step(p, rpm, u, dt)
  p.t = p.t + dt
  if p.prev_u ~= nil and u * p.prev_u < 0 then
    p.nswitch = p.nswitch + 1
    if p.nswitch > 2 then
      plant_close(p)
    end
    plant_begin(p, rpm, u)
  elseif p.prev_u == nil then
    plant_begin(p, rpm, u)
  elseif not p.turned then
    if math.abs(rpm - p.switch_rpm) >= math.abs(p.peak_rpm - p.switch_rpm) then
      p.peak_rpm = rpm
      p.peak_t = p.t
    elseif math.abs(p.peak_rpm - p.switch_rpm) >= p.turn_rpm and
        math.abs(rpm - p.peak_rpm) >= p.turn_rpm then
      p.turned = true
      p.ramp_t = p.t
      p.ramp_rpm = rpm
    end
  elseif p.kv_t == nil and p.ramp_t ~= nil and (p.t - p.ramp_t) >= 0.06 then
    p.kv_rpm = rpm
    p.kv_t = p.t
  end
  p.prev_u = u
  p.prev_rpm = rpm
end

local function plant_flush(p)
  if p.nswitch > 2 then
    plant_close(p)
  end
end

-- Prefer the measured Kv and T. Fall back to the fundamental Ku and Pu.
local function wheel_gains(plant, relay, amps, k)
  local a, aerr = robust_median(amps, 0.25, 4)
  local pu, puerr = robust_median(relay.periods, 0.25, 4)
  if aerr == "spread" or puerr == "spread" then
    return nil, "inconsistent cycles"
  end
  if a == nil or pu == nil then
    return nil, "not enough cycles"
  end
  local h = relay.hyst_rad
  if a < 4 * h then
    return nil, "oscillation too small for hysteresis"
  end
  local under = a * a - h * h
  if under <= 0 then
    return nil, "oscillation too small for hysteresis"
  end
  local ku = 4 * relay.relay / (math.pi * math.sqrt(under))
  local kv = robust_median(plant.kvs, 0.35, 3)
  local lag = robust_median(plant.ts, 0.35, 3)
  local kp, ki, how
  if kv ~= nil and lag ~= nil then
    kp, ki = gains_type2(kv, lag)
    how = "type2"
  end
  if not gains_sane(kp, ki, k) then
    kp, ki = gains_fallback(ku, pu)
    how = "fallback"
    kv = kv or (4 * a / (relay.relay * pu))
    lag = lag or (pu / 4)
  end
  if not gains_sane(kp, ki, k) then
    return nil, "gains out of range"
  end
  return { P = kp, I = ki, Ku = ku, Pu = pu, Kv = kv, T = lag, how = how }, nil
end

local function announce_fit(name, g)
  if g.how == "type2" then
    tune_message(MAV_SEVERITY.INFO, string.format("%s type2 Kv %.0f T %.3f", name, g.Kv, g.T))
  else
    tune_message(MAV_SEVERITY.INFO, string.format("%s fallback Ku %.3f Pu %.3f", name, g.Ku, g.Pu))
  end
  tune_message(MAV_SEVERITY.INFO, string.format("%s P %.4f I %.4f", name, g.P, g.I))
end

-- Hysteresis sits above the hold noise and stays far below the old 3% of ESC_RPM.
local function hyst_of(sigma, target)
  local cap = math.max(0.05 * math.abs(target), 6)
  return constrain(3 * (sigma or 0), 2, cap)
end

local function sigma_of(sum, sum2, n)
  if n < 2 then
    return 0
  end
  local mean = sum / n
  local v = sum2 / n - mean * mean
  if v < 0 then
    return 0
  end
  return math.sqrt(v)
end

-- Current per rpm from two holds. A negative slope falls back to the high point.
local function ff_slope(i_lo, rpm_lo, i_hi, rpm_hi)
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
local function relay_amp(k)
  local frac = constrain(g("WT_RELAY"), 1, 100) / 100
  return math.max(k.max_current * frac, 0.5)
end

-- modules/autotune.lua names its limits *_rad after the yaw use; here they are in rpm.
-- The module's own spread check is loose. wheel_gains applies the 25% median test
-- to the Fourier amplitude and to Pu. kp_div / ti_mul match the h = 7 fallback.
local function make_relay(k, hyst, max_frac)
  local amp = relay_amp(k)
  return Autotune:new({
    relay = amp,
    hyst_rad = hyst,
    max_amp_rad = k.max_rpm * (max_frac or 0.6),
    skip_cycles = 3,
    cycles = 6,
    timeout_s = 15,
    min_period_s = 0.02,
    max_period_s = 2,
    max_spread = 2,
    min_amp_ratio = 1.5,
    kp_div = 1 / 0.36,
    ti_mul = 1.75,
  })
end

local function zero_currents()
  return { 0, 0, 0, 0, 0, 0, 0, 0 }
end

-- The speed test is fixed at 120 rpm, with the low plateau at half of that.
local TUNE_HI_RPM = 120

local function tune_levels(k)
  local hi = TUNE_HI_RPM
  local lo = hi * 0.5
  if hi > k.max_rpm then
    hi = k.max_rpm
    lo = hi * 0.5
  end
  return lo, hi
end

local function tune_mag(k, level)
  local lo, hi = tune_levels(k)
  if level == 1 and lo ~= nil then
    return lo, true
  end
  return hi, lo ~= nil
end

local function clear_wheel_pi()
  for i = 1, K.MAX_FB do
    esc[i].integrator = 0
    esc[i].last_error = 0
    esc[i].ref_rpm = 0
  end
end

local function announce_saved(ps, is, fs)
  tune_message(MAV_SEVERITY.NOTICE, M.fmt4("P", ps))
  tune_message(MAV_SEVERITY.NOTICE, M.fmt4("I", is))
  tune_message(MAV_SEVERITY.NOTICE, M.fmt4("FF", fs))
  tune_message(MAV_SEVERITY.NOTICE, "SCR_USER1 set to 0 (normal)")
end

local function tune_reset_window(zero_ref)
  tune.t = 0
  tune.count = 0
  tune.sum_current = { 0, 0, 0, 0 }
  tune.sum_rpm = { 0, 0, 0, 0 }
  tune.sum_mean = 0
  tune.sum_mean2 = 0
  if zero_ref then
    clear_wheel_pi()
  else
    for i = 1, K.MAX_FB do
      esc[i].integrator = 0
      esc[i].last_error = 0
    end
  end
end

local function tune_start_direction(dir)
  tune.dir = dir
  tune.state = "ff"
  tune.ff_level = 1
  tune.lo_i = nil
  tune.lo_rpm = nil
  tune.base = { 0, 0, 0, 0 }
  tune.rpm = { 0, 0, 0, 0 }
  tune.relay = nil
  tune.fourier = nil
  tune.plant = nil
  tune.prev_cycle = 0
  tune_reset_window(true)
end

local function tune_finish()
  local ok, ps, is_, fs = M.save_wheel_gains(tune.results)
  if not ok then
    tune_fail(ps)
    return
  end
  local lo, hi = nil, nil
  for w = 1, K.NUM_ESC do
    if ps[w] ~= nil then
      lo = (lo == nil) and ps[w] or math.min(lo, ps[w])
      hi = (hi == nil) and ps[w] or math.max(hi, ps[w])
    end
  end
  if lo ~= nil and hi > lo * 1.3 then
    tune_message(MAV_SEVERITY.WARNING, "wheel P differs by more than 30%")
  end
  M.queue_save(scr_user1, 0)
  clear_wheel_pi()
  tune.state = "done"
  announce_saved(ps, is_, fs)
end

local function tune_log_wheels(target, cur)
  if logger == nil then
    return
  end
  local ph = tune.state == "relay" and 2 or 1
  local cyc = tune.relay and tune.relay.cycle or 0
  for i = 1, K.NUM_ESC do
    if esc[i].detected then
      logger:write("DJWT", "Side,Dir,Ph,Tgt,Rpm,Cur,Cyc", "BbBfffB",
        i, tune.dir or 0, ph, target, esc[i].wheel_rpm, cur[i], cyc)
    end
  end
end

-- One PI from the mean plant. Each wheel keeps its own current slope as FF.
local function finish_synced_relay(k)
  plant_flush(tune.plant)
  local g, err = wheel_gains(tune.plant, tune.relay, tune.fourier.amps, k)
  if g == nil then
    tune_fail(err)
    return false
  end
  local row = {}
  for w = 1, K.NUM_ESC do
    if esc[w].detected then
      local lo_i = tune.lo_i and tune.lo_i[w] or nil
      local lo_rpm = tune.lo_rpm and tune.lo_rpm[w] or nil
      local ff = ff_slope(lo_i, lo_rpm, tune.base[w], tune.rpm[w])
      if ff == nil or not finite(ff) or ff < 0 then
        tune_fail(string.format("ESC %d rpm too low for FF", w))
        return false
      end
      row[w] = { P = g.P, I = g.I, Ku = g.Ku, Pu = g.Pu, ff = ff }
    end
  end
  tune.results[#tune.results + 1] = row
  announce_fit(tune.dir > 0 and "fwd" or "rev", g)
  if tune.dir > 0 then
    tune_start_direction(-1)
  else
    tune_finish()
  end
  return true
end

--[[
Synchronised relay (SCR_USER1 = 4). Returns per-wheel currents while it owns
the outputs, or nil to let the normal speed loop run.

Phase "ff":    hold 60 rpm, then 120 rpm. The current slope of each wheel
               is ESC_FFn. The high hold also measures rpm noise for the hysteresis.
Phase "relay": that high-point current plus one common relay, switched on the
               mean rpm error. Kv and T of the mean, or the fundamental Ku and Pu,
               set one PI copied to every wheel.
Forward first, then reverse.
--]]
local function wheel_tune(dt, k, armed, enabled)
  local wanted = tune_wanted(armed)
  if not wanted then
    if tune.state == "ff" or tune.state == "relay" then
      tune_message(MAV_SEVERITY.WARNING, "aborted (left HOLD, disarmed or selector changed)")
    end
    if tune.state ~= "idle" then
      tune_reset()
      clear_wheel_pi()
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
    local lo, hi = tune_levels(k)
    local speeds = lo and string.format("%.0f/%.0f", lo, hi) or string.format("%.0f", hi)
    tune_message(MAV_SEVERITY.NOTICE, string.format("synced; %s rpm fwd/rev relay %.1fA",
      speeds, relay_amp(k)))
  end
  if not enabled then
    tune_fail("ESC feedback lost or outputs disabled")
    return nil
  end
  if not rc_centered() then
    tune_fail("stick moved")
    return nil
  end

  local mag = tune_mag(k, tune.ff_level)
  local target = tune.dir * mag
  local cur = zero_currents()
  tune.t = tune.t + dt

  if tune.state == "ff" then
    local any = false
    local rpm_sum, rpm_n = 0, 0
    for i = 0, K.NUM_ESC - 1 do
      local fb = esc[i + 1]
      if fb.detected then
        any = true
        profile_wheel(i, target, dt)
        cur[i + 1] = wheel_pi(i, fb.ref_rpm, dt, k)
        rpm_sum = rpm_sum + fb.wheel_rpm
        rpm_n = rpm_n + 1
        if tune.t >= K.TUNE_SETTLE_S then
          tune.sum_current[i + 1] = tune.sum_current[i + 1] + cur[i + 1]
          tune.sum_rpm[i + 1] = tune.sum_rpm[i + 1] + fb.wheel_rpm
        end
      end
    end
    if not any then
      tune_fail("no ESC detected")
      return zero_currents()
    end
    if tune.t >= K.TUNE_SETTLE_S and rpm_n > 0 then
      local mean_rpm = rpm_sum / rpm_n
      tune.sum_mean = tune.sum_mean + mean_rpm
      tune.sum_mean2 = tune.sum_mean2 + mean_rpm * mean_rpm
      tune.count = tune.count + 1
    end
    if tune.t >= K.TUNE_SETTLE_S + K.TUNE_AVERAGE_S then
      if tune.count < 1 then
        tune_fail("no samples")
        return zero_currents()
      end
      local means_i, means_r = {}, {}
      for i = 0, K.NUM_ESC - 1 do
        if esc[i + 1].detected then
          local mean_rpm = tune.sum_rpm[i + 1] / tune.count
          if math.abs(mean_rpm - target) > 0.3 * math.abs(target) then
            tune_fail(string.format("ESC %d did not reach %.0f rpm (%.0f)", i + 1, target, mean_rpm))
            return zero_currents()
          end
          means_i[i + 1] = tune.sum_current[i + 1] / tune.count
          means_r[i + 1] = mean_rpm
          esc[i + 1].ref_rpm = target
        end
      end
      local _, has_lo = tune_mag(k, tune.ff_level)
      if tune.ff_level == 1 and has_lo then
        tune.lo_i = means_i
        tune.lo_rpm = means_r
        tune.ff_level = 2
        tune_reset_window(false)
      else
        tune.base = means_i
        tune.rpm = means_r
        local hyst = hyst_of(sigma_of(tune.sum_mean, tune.sum_mean2, tune.count), target)
        tune.relay = make_relay(k, hyst)
        tune.relay:start()
        tune.fourier = new_fourier()
        tune.plant = new_plant(hyst)
        tune.prev_cycle = 0
        tune.state = "relay"
        tune.t = 0
      end
    end
  elseif tune.state == "relay" then
    local err_sum, err_n = 0, 0
    for i = 0, K.NUM_ESC - 1 do
      local fb = esc[i + 1]
      if fb.detected then
        local err = target - fb.wheel_rpm
        fb.target_rpm = target
        err_sum = err_sum + err
        err_n = err_n + 1
      end
    end
    local mean_err = (err_n > 0) and (err_sum / err_n) or 0
    local u, status = tune.relay:step(mean_err, dt)
    if status == "fail" then
      tune_fail(tune.relay:result().msg)
      return zero_currents()
    end
    tune.prev_cycle = track_cycle(tune.relay, tune.fourier, mean_err, dt, tune.prev_cycle)
    if status ~= "done" then
      plant_step(tune.plant, target - mean_err, u, dt)
    end
    for i = 0, K.NUM_ESC - 1 do
      if esc[i + 1].detected then
        cur[i + 1] = constrain(tune.base[i + 1] + u, -k.max_current, k.max_current)
      end
    end
    if status == "done" then
      if not finish_synced_relay(k) then
        return zero_currents()
      end
      return zero_currents()
    end
  end
  tune_log_wheels(target, cur)
  return cur
end

--[[
Per-wheel relay (SCR_USER1 = 5), one wheel at a time, others at 0 A.
Wheels must be in the air: on the ground one driven wheel yaws the vehicle and
the identified plant is no longer that motor. SCR_USER1 = 6 is retired.
--]]
local LT = { state = "idle" }

function LT.fail(msg)
  LT.state = "fail"
  tune_message(MAV_SEVERITY.ERROR, "failed - " .. msg .. "; parameters kept")
end

function LT.next_wheel(from)
  for i = from, K.NUM_ESC - 1 do
    if esc[i + 1].detected then
      return i
    end
  end
  return nil
end

function LT.begin_window(zero_ref)
  LT.t = 0
  LT.sum_c = 0
  LT.sum_r = 0
  LT.sum_r2 = 0
  LT.count = 0
  if zero_ref then
    clear_wheel_pi()
    return
  end
  local fb = esc[LT.idx + 1]
  fb.integrator = 0
  fb.last_error = 0
end

function LT.arm_hold()
  LT.state = "hold"
  LT.level = 1
  LT.relay = nil
  LT.fourier = nil
  LT.plant = nil
  LT.i_lo = nil
  LT.rpm_lo = nil
  LT.begin_window(true)
end

function LT.one_wheel(dt, k)
  local cur = zero_currents()
  local idx = LT.idx
  local mag = tune_mag(k, LT.level or 1)
  local target = LT.dir * mag
  if LT.state == "rest" then
    if LT.t >= K.BRK_REST_S then
      LT.arm_hold()
    end
    return cur
  end
  if LT.state == "hold" then
    local fb = esc[idx + 1]
    profile_wheel(idx, target, dt)
    cur[idx + 1] = wheel_pi(idx, fb.ref_rpm, dt, k)
    if LT.t >= K.TUNE_SETTLE_S then
      LT.sum_c = LT.sum_c + cur[idx + 1]
      LT.sum_r = LT.sum_r + fb.wheel_rpm
      LT.sum_r2 = LT.sum_r2 + fb.wheel_rpm * fb.wheel_rpm
      LT.count = LT.count + 1
    end
    if LT.t >= K.TUNE_SETTLE_S + K.TUNE_AVERAGE_S then
      if LT.count < 1 then
        LT.fail(string.format("ESC %d no samples", idx + 1))
        return cur
      end
      local mean_rpm = LT.sum_r / LT.count
      if math.abs(mean_rpm - target) > 0.3 * math.abs(target) then
        LT.fail(string.format("ESC %d did not reach %.0f rpm (%.0f)", idx + 1, target, mean_rpm))
        return cur
      end
      local _, has_lo = tune_mag(k, LT.level)
      if LT.level == 1 and has_lo then
        LT.i_lo = LT.sum_c / LT.count
        LT.rpm_lo = mean_rpm
        LT.level = 2
        fb.ref_rpm = target
        LT.begin_window(false)
        return cur
      end
      LT.base = LT.sum_c / LT.count
      LT.mean_rpm = mean_rpm
      fb.ref_rpm = target
      local hyst = hyst_of(sigma_of(LT.sum_r, LT.sum_r2, LT.count), target)
      LT.relay = make_relay(k, hyst, 1.0)
      LT.relay:start()
      LT.fourier = new_fourier()
      LT.plant = new_plant(hyst)
      LT.prev_cycle = 0
      LT.state = "relay"
      LT.t = 0
    end
    return cur
  end
  local fb = esc[idx + 1]
  fb.target_rpm = target
  local err = target - fb.wheel_rpm
  local u, status = LT.relay:step(err, dt)
  if status == "fail" then
    LT.fail(string.format("ESC %d: %s", idx + 1, LT.relay:result().msg))
    return cur
  end
  cur[idx + 1] = constrain(LT.base + u, -k.max_current, k.max_current)
  LT.prev_cycle = track_cycle(LT.relay, LT.fourier, err, dt, LT.prev_cycle)
  if status ~= "done" then
    plant_step(LT.plant, fb.wheel_rpm, u, dt)
    return cur
  end
  plant_flush(LT.plant)
  local g, why = wheel_gains(LT.plant, LT.relay, LT.fourier.amps, k)
  if g == nil then
    LT.fail(string.format("ESC %d: %s", idx + 1, why))
    return cur
  end
  g.ff = ff_slope(LT.i_lo, LT.rpm_lo, LT.base, LT.mean_rpm)
  if g.ff == nil or not finite(g.ff) or g.ff < 0 then
    LT.fail(string.format("ESC %d rpm too low for FF", idx + 1))
    return cur
  end
  LT.results[LT.dir > 0 and 1 or 2][idx + 1] = g
  announce_fit(string.format("ESC%d %s", idx + 1, LT.dir > 0 and "fwd" or "rev"), g)
  if LT.dir > 0 then
    LT.dir = -1
    LT.state = "rest"
    LT.t = 0
    return cur
  end
  local nxt = LT.next_wheel(idx + 1)
  if nxt == nil then
    local ok, ps, is_, fs = M.save_wheel_gains(LT.results)
    if not ok then
      LT.fail(ps)
      return cur
    end
    M.queue_save(scr_user1, 0)
    clear_wheel_pi()
    LT.state = "done"
    announce_saved(ps, is_, fs)
    return cur
  end
  LT.idx = nxt
  LT.dir = 1
  LT.state = "rest"
  LT.t = 0
  tune_message(MAV_SEVERITY.INFO, string.format("next ESC %d", nxt + 1))
  return cur
end

function LT.step(dt, k, armed, enabled)
  local sel = selector()
  if sel == K.LOC_SELECTOR then
    local now = now_ms()
    if LT.note_ms == nil or now - LT.note_ms >= 10000 then
      LT.note_ms = now
      tune_message(MAV_SEVERITY.WARNING, "USER1=6 retired; PI is per wheel now")
    end
    return nil
  end
  local mode = (sel == K.BRK_SELECTOR) and sel or nil
  local running = LT.state == "rest" or LT.state == "hold" or LT.state == "relay"
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
    if not k.speed_mode then
      LT.fail("ESC_MODE must be 0")
      return nil
    end
    LT.mode = mode
    LT.results = { {}, {} }
    LT.idx = LT.next_wheel(0)
    if LT.idx == nil then
      LT.fail("no ESC detected")
      return nil
    end
    LT.dir = 1
    tune_message(MAV_SEVERITY.NOTICE, "in the air; one wheel at a time")
    LT.arm_hold()
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
  return LT.one_wheel(dt, k)
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

  local wheel_currents = wheel_tune(dt, k, armed, enabled)
  if wheel_currents == nil then
    wheel_currents = LT.step(dt, k, armed, enabled)
  end
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
        if math.abs(sd.interp_value) < 0.01 and math.abs(fb.ref_rpm) < K.MOVING_RPM
            and math.abs(fb.wheel_rpm) < K.MOVING_RPM then
          fb.integrator = 0
          fb.last_error = 0
        end
        fb.current_a = wheel_pi(i, fb.ref_rpm, dt, k)
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

local function publish_stat()
  local stat = 0
  if latched_can and driver ~= nil then
    stat = stat | K.STAT_CAN
  end
  if sat_ms ~= nil and now_ms() - sat_ms < K.SAT_HOLD_MS then
    stat = stat | K.STAT_SAT
  end
  if tune.state == "ff" or tune.state == "relay" or
    LT.state == "rest" or LT.state == "hold" or LT.state == "relay" then
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

local function push_telem()
  local data = M.telem
  for i = 0, K.NUM_ESC - 1 do
    local fb = esc[i + 1]
    if fb.detected and healthy(i) then
      data:motor_temp_cdeg(math.floor(constrain(fb.temperature, 0, 200) * 100))
      data:current(math.abs(fb.current_raw) / K.RAW_PER_AMP)
      esc_telem:update_telem_data(i, data, K.TELEM_MOTOR_TEMP | K.TELEM_CURRENT)
    end
  end
end

local function send_telem()
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
  pcall(push_telem)
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
  -- Same cadence as the CAN build where the C620 stayed on the bus:
  -- zero MAIN 1..4 before RX and again after the current command, every 5 ms.
  if latched_can then
    kill_pwm()
  end
  drain()
  run_control()
  if latched_can then
    kill_pwm()
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
