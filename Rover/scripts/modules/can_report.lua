--[[
CAN driver reports: DJIC log, ESC telemetry, the live NAMED_VALUE_FLOAT stream
(ESC_GCS_HZ), the receive counters while an ESC is missing, and the warning
for a wheel tune selector without CAN. Part of the CAN output (can_esc.lua).
--]]
if ... == nil then
  return
end

local R = require("rover_common")
local C = require("can_state")
local K, P, M, esc, side, slip_since = C.K, C.P, C.M, C.esc, C.side, C.slip_since
local MAV_SEVERITY = R.MAV_SEVERITY
local as_number, now_ms, constrain, finite, g = C.as_number, C.now_ms, C.constrain, C.finite, C.g
local set_param, direction, side_of, mask_valid, is_throttle = C.set_param, C.direction, C.side_of, C.mask_valid, C.is_throttle
local script_mixer_active, selector, healthy, all_healthy = C.script_mixer_active, C.selector, C.healthy, C.all_healthy
local kill_pwm, reset_command, reset_control, tune_message, tune_reset = C.kill_pwm, C.reset_command, C.reset_control, C.tune_message, C.tune_reset
local send_group, reset_esc, reset_side = C.send_group, C.reset_esc, C.reset_side
local scr_user1, scr_user5, rc1_trim, rc3_trim = C.scr_user1, C.scr_user5, C.rc1_trim, C.rc3_trim
local diag = C.diag
local PARAM_SPEC = C.PARAM_SPEC

function M.write_log()
  if (M.gains.opt & 1) == 0 or logger == nil then
    return
  end
  local now = now_ms()
  if now - C.last_log < 50 then
    return
  end
  C.last_log = now
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
local function gcs_push(name, value)
  local n = M.gcs_q_n + 1
  M.gcs_q_n = n
  M.gcs_q_name[n] = name
  M.gcs_q_val[n] = value
end

function M.send_gcs()
  local hz = M.gains.gcs_hz
  if hz <= 0 or not C.latched_can then
    M.gcs_q_n = 0
    return
  end
  -- Finish the previous sample before taking another, one message per loop.
  if M.gcs_q_i <= M.gcs_q_n then
    gcs:send_named_float(M.gcs_q_name[M.gcs_q_i], M.gcs_q_val[M.gcs_q_i])
    M.gcs_q_i = M.gcs_q_i + 1
    return
  end
  local now = now_ms()
  if now - M.gcs_ms < 1000 / hz then
    return
  end
  M.gcs_ms = now
  M.gcs_q_n = 0
  M.gcs_q_i = 1
  -- Motor temperature (C620 feedback byte 6, deg C) once per second: it moves slowly
  local temp = now - M.gcs_temp_ms >= 1000
  if temp then
    M.gcs_temp_ms = now
  end
  for i = 1, K.NUM_ESC do
    local fb = esc[i]
    if fb.detected then
      -- Constant names: no new strings on every send
      local names = K.GCS_NAMES[i]
      gcs_push(names[1], direction(i - 1) * fb.current_raw / K.RAW_PER_AMP)
      gcs_push(names[2], fb.current_a)
      gcs_push(names[3], fb.wheel_rpm)
      if temp then
        gcs_push(names[4], fb.temperature)
      end
    end
  end
  gcs_push("ESC_TGT_L", side[1].target_rpm or 0)
  gcs_push("ESC_TGT_R", side[2].target_rpm or 0)
end

function M.send_telem()
  if esc_telem == nil or now_ms() - C.last_telem < 20 then
    return
  end
  C.last_telem = now_ms()
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
  if C.latched_can and missing and not arming:is_armed() then
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
  if C.latched_can or (sel ~= K.WHEEL_TUNE_SELECTOR and sel ~= K.BRK_SELECTOR and sel ~= K.LOC_SELECTOR) then
    return
  end
  local now = now_ms()
  if now - C.last_tune_warn >= 10000 then
    C.last_tune_warn = now
    tune_message(MAV_SEVERITY.WARNING, string.format("SCR_USER1=%d needs ESC_CAN=1; nothing done", math.floor(sel)))
  end
end

return true
