--[[
CAN driver shared state: constants, the ESC_ parameter table, per-wheel and
per-side state, and the helpers every can_* module uses.
Part of the CAN output (can_esc.lua). Remove all can_*.lua files to run on PWM only.
--]]
if ... == nil then
  return
end

local R = require("rover_common")
local C = {}

local K = {
  -- Shown in "ESC: ready" so a test can confirm which file is on the SD card
  VERSION = "v2611p",
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
  -- Smallest speed target the stall boost acts on, and the side command below
  -- which a stopped side coasts at 0 A
  STALL_MIN_RPM = 0.5,
  -- A wheel under this rpm counts as stalled even for a tiny target: the rpm
  -- reading jitters about 0.4 rpm and switched the boost on and off
  STALL_NOISE_RPM = 1,
  REST_CMD = 0.004,
  -- Share of the stall boost removed once the wheel breaks free
  STALL_RELEASE = 0.7,
  -- Traction control (ESC_TC): a wheel faster than the other wheel of its side
  -- by more than ESC_TC_RPM or TC_SPREAD of the target (the larger), for
  -- TC_DETECT_MS, is free; the control ends when the loaded wheel reaches
  -- TC_RELEASE of the side target
  TC_DETECT_MS = 150,
  TC_SPREAD = 0.3,
  TC_RELEASE = 0.7,
  -- The loaded wheel must stay above TC_RELEASE this long before the control
  -- ends: right after the full current it often drops back and stalls again
  TC_RELEASE_MS = 1000,
  TC_WARN_MS = 3000,
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
  STAT_TC = 64,
  PARAM_KEY = 74,
  PREFIX = "ESC_",
  TELEM_CURRENT = 1 << 3,
  TELEM_MOTOR_TEMP = 1 << 1,
  GCS_NAMES = {
    { "ESC1_I", "ESC1_ICMD", "ESC1_RPM", "ESC1_T" },
    { "ESC2_I", "ESC2_ICMD", "ESC2_RPM", "ESC2_T" },
    { "ESC3_I", "ESC3_ICMD", "ESC3_RPM", "ESC3_T" },
    { "ESC4_I", "ESC4_ICMD", "ESC4_RPM", "ESC4_T" },
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
  { 45, "WT_BW", 5 },
  { 46, "STALL", 30 },
  -- Append only: existing parameter slots must keep their saved meaning.
  --[[
    // @Param: ESC_RPM_AUX
    // @DisplayName: RPM control auxiliary features
    // @Description: Enable configured RPM reference ramp, prefilter, opposing-current restriction, stall boost and side desaturation. Zero bypasses these helpers without changing their saved settings. PID, absolute current limits, anti-windup and safety stops remain active. Also applies to wheel autotune verification. Effective within 50 ms.
    // @Values: 0:Disabled,1:Enabled
    // @Range: 0 1
    // @User: Standard
  --]]
  { 47, "RPM_AUX", 1 },
  --[[
    // @Param: ESC_TC
    // @DisplayName: Traction control
    // @Description: When one wheel of a side spins free (lifted, no load) while the other wheel of the same side holds the rover, cap the free wheel at the loaded wheel's rpm plus ESC_TC_RPM and let the loaded wheel use up to ESC_CUR instead of ESC_IMAX. Ends when the loaded wheel reaches 70% of the side target. Off with ESC_RPM_AUX = 0.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
  --]]
  { 48, "TC", 1 },
  --[[
    // @Param: ESC_TC_RPM
    // @DisplayName: Traction control margin
    // @Description: Wheel rpm the free wheel may run above the loaded wheel of its side. A difference larger than this, or than 30% of the side target, for 150 ms starts the traction control.
    // @Units: rpm
    // @Range: 2 100
    // @User: Standard
  --]]
  { 49, "TC_RPM", 15 },
}

local MAV_SEVERITY = R.MAV_SEVERITY
C.K = K
C.PARAM_SPEC = PARAM_SPEC

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
C.P = P
C.params_ready = false
local scr_user1 = Parameter()
local scr_user5 = Parameter()
C.scr_user1 = scr_user1
C.scr_user5 = scr_user5
C.rc1_trim = Parameter()
C.rc3_trim = Parameter()
C.user_params_bound = false

C.driver = nil
-- Stay in PWM until ESC_CAN is read. Starting as CAN would zero MAIN 1..4.
C.latched_can = false
C.auth_id = nil
C.last_switch_warn = 0
C.last_lost_warn = 0
C.last_driver_warn = 0
C.last_mask_warn = 0
C.diag = {n = {0, 0, 0, 0, 0, 0, 0, 0}, other = 0, other_id = 0, last_ms = 0, last_loop_ms = 0, max_dt = 0}
C.last_slip_warn = 0
C.last_tune_warn = 0
C.last_log = 0
C.last_telem = 0
C.last_us = nil
C.lost_mask = 0
C.announced_driver = false
C.published_stat = -1
C.sat_ms = nil
C.slip_active = false

local esc = {}
C.esc = esc
local slip_since = {}
C.slip_since = slip_since
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
    boost = 0,
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
C.side = side
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

-- Commands from the mixer (mixer.lua calls can_esc's command())
C.cmd_l = 0
C.cmd_r = 0
C.cmd_seq = 0
C.cmd_seq_seen = nil
C.cmd_seq_ms = nil

C.tune = { state = "idle" }

local as_number = R.as_number
local now_ms = R.now_ms
local constrain = R.constrain
local finite = R.finite

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
  gcs_temp_ms = 0,
  -- One NAMED_VALUE_FLOAT per loop. A burst of all four motors overflows the
  -- MAVLink queue and the later wheels arrive at a fraction of ESC_GCS_HZ.
  gcs_q_name = {},
  gcs_q_val = {},
  gcs_q_n = 0,
  gcs_q_i = 1,
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
    stall = 0,
  },
  scratch = {
    sum = { 0, 0 },
    count = { 0, 0 },
    lag = {},
    fast = {},
    raw = { 0, 0, 0, 0, 0, 0, 0, 0 },
    goals = { 0, 0 },
    -- Wheel index (0-based) of the slowest / fastest wheel of each side
    lag_i = { 0, 0 },
    fast_i = { 0, 0 },
  },
}

C.M = M

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

function M.zero_currents()
  return { 0, 0, 0, 0, 0, 0, 0, 0 }
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

local function script_mixer_active()
  return C.user_params_bound and scr_user5:get() == 2
end

local function selector()
  if not C.user_params_bound then
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
  C.cmd_seq_seen = nil
  C.cmd_seq_ms = nil
end

local function reset_control()
  reset_side(1)
  reset_side(2)
  for i = 1, K.MAX_FB do
    local fb = esc[i]
    fb.current_a = 0
    fb.integrator = 0
    fb.boost = 0
    fb.last_error = 0
    fb.ref_rpm = 0
    fb.ref_f = 0
    fb.target_rpm = 0
    fb.saturated = false
    fb.tc_full = false
  end
end

local function tune_message(severity, text)
  gcs:send_text(severity, "WheelTune: " .. text)
end

local function tune_reset()
  C.tune = { state = "idle" }
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
  C.driver:write_frame(frame, 1000)
end

C.as_number = as_number
C.now_ms = now_ms
C.constrain = constrain
C.finite = finite
C.g = g
C.set_param = set_param
C.reset_esc = reset_esc
C.reset_side = reset_side
C.direction = direction
C.side_of = side_of
C.mask_valid = mask_valid
C.is_throttle = is_throttle
C.script_mixer_active = script_mixer_active
C.selector = selector
C.healthy = healthy
C.all_healthy = all_healthy
C.kill_pwm = kill_pwm
C.reset_command = reset_command
C.reset_control = reset_control
C.tune_message = tune_message
C.tune_reset = tune_reset
C.send_group = send_group

return C
