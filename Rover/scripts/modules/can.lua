-- C620 / M3508 driver for stock ArduPilot Rover.
-- Install in APM/scripts/modules alongside the other mixer modules.
--
-- CAN_Px_DRIVER = 1 or 2
-- CAN_Px_BITRATE = 1000000
-- CAN_Dx_PROTOCOL = 10 (Scripting)
-- SCR_ENABLE = 1
-- SCR_USER5 = 2 enables the rover mixer.
--
-- CAN_ENABLE 0 = PWM, 1 = CAN. Change only while disarmed.
-- CAN_STEP_MIN and CAN_STEP_INC set the initial current and increment in amps.
-- CAN_STEP_TIME sets the duration of each current plateau in seconds.
-- CAN_RESP_ESC: 1-4 selects one ESC, 5 tests 1-4, 6 tests 4-1.
-- CAN_GEAR, CAN_CUR, CAN_FF, CAN_IMAX, CAN_REV, CAN_OPT
-- CAN_P_1/CAN_I_1/CAN_D_1 through CAN_P_4/CAN_I_4/CAN_D_4 tune each ESC.
--
-- Wheel distance is logged in DJIC.Pos. Stock firmware has no Lua hook into
-- WENC, so leave WRC_ENABLE = 0. The speed loop below is the closed loop.

local MAX_ESC = 4
local FULL_SCALE_RPM = 400
local CMD_1_4 = 0x200
local FB_BASE = 0x200
local COUNTS_PER_REV = 8192
local RAW_PER_AMP = 16384 / 20
local RAW_MAX = 16384
local FEEDBACK_TIMEOUT_MS = 100
local LOOP_MS = 5
local MAX_FRAMES = 25
local rx_frames = 0
-- Keep the original CAN key when available so saved settings still load.
local PARAM_KEYS = { 19, 93, 94 }
local DEFAULT_STEP_MIN = 0.2
local DEFAULT_STEP_INC = 0.1
local DEFAULT_STEP_TIME = 5
local DEFAULT_TUNE_STEP_TIME_S = 5
local TELEM_CURRENT = 1 << 3
local TELEM_MOTOR_TEMP = 1 << 1

local MAV_SEVERITY = { CRITICAL = 2, WARNING = 4, NOTICE = 5, INFO = 6 }

local driver = nil
local latched_can = false
local auth_id = nil
local last_switch_warn = uint32_t(0)
local last_lost_warn = uint32_t(0)
local last_driver_warn = uint32_t(0)
local last_log = uint32_t(0)
local last_telem = uint32_t(0)
local params_ready = false
local param_table_failure_reported = false
local can_api = CAN
local last_us = nil
local lost_mask = 0
local announced_driver = false

local script_norm = { 0, 0, 0, 0 }
local script_ms = uint32_t(0)
local script_fresh = false

local esc = {}
for i = 1, MAX_ESC do
  esc[i] = {
    angle = 0,
    have_angle = false,
    rotor_counts = 0,
    rotor_rpm = 0,
    current_raw = 0,
    temperature = 0,
    last_ms = nil,
    detected = false,
    integrator = 0,
    last_error = 0,
    target_rpm = 0,
    wheel_rpm = 0,
    current_a = 0,
  }
end

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

local function pget(name, default)
  local value = param:get(name)
  if value == nil then
    return default
  end
  return value
end

local function ensure_params()
  if params_ready then
    return true
  end
  local param_key
  for _, key in ipairs(PARAM_KEYS) do
    if param:add_table(key, "CAN_", 38) then
      param_key = key
      break
    end
  end
  if param_key == nil then
    if not param_table_failure_reported then
      gcs:send_text(MAV_SEVERITY.WARNING, "DJICAN: CAN_ table rejected (keys 19, 93, 94)")
      param_table_failure_reported = true
    end
    return false
  end
  local spec = {
    { 1, "ENABLE", 0 },
    { 2, "STEP_RESP", 0 },
    { 3, "GEAR", 19.2032 },
    { 4, "CUR", 10 },
    { 5, "FF", 0 },
    { 6, "IMAX", 10 },
    { 7, "REV", 0 },
    { 8, "OPT", 1 },
  }
  for i = 1, MAX_ESC do
    local base = 8 + (i - 1) * 3
    spec[#spec + 1] = { base + 1, "P_" .. i, 0.03 }
    spec[#spec + 1] = { base + 2, "I_" .. i, 0.3 }
    spec[#spec + 1] = { base + 3, "D_" .. i, 0 }
  end
  -- Register in slot order. AP_Param marks skipped slots as hidden.
  spec[#spec + 1] = { 21, "STEP_RPM", 0 }
  spec[#spec + 1] = { 22, "STEP_MIN", DEFAULT_STEP_MIN }
  spec[#spec + 1] = { 23, "STEP_TIME", DEFAULT_STEP_TIME }
  spec[#spec + 1] = { 24, "STEP_INC", DEFAULT_STEP_INC }
  spec[#spec + 1] = { 25, "RESP_ESC", 6 }
  for i = 1, MAX_ESC do
    spec[#spec + 1] = { 28 + 2 * i, "START_F_" .. i, 0 }
    spec[#spec + 1] = { 29 + 2 * i, "START_R_" .. i, 0 }
  end
  spec[#spec + 1] = { 38, "TUNE_TIME", DEFAULT_TUNE_STEP_TIME_S }
  for n = 1, #spec do
    local item = spec[n]
    if not param:add_param(param_key, item[1], item[2], item[3]) then
      gcs:send_text(MAV_SEVERITY.WARNING, string.format("DJICAN: failed to add CAN_%s", item[2]))
      return false
    end
  end
  params_ready = true
  gcs:send_text(MAV_SEVERITY.INFO, string.format("DJICAN: CAN_* parameters ready (key %d)", param_key))
  gcs:send_text(MAV_SEVERITY.NOTICE, "DJICAN: CAN_RESP_ESC ready (1-6)")
  return true
end

local function direction(idx)
  local rev = math.floor(pget("CAN_REV", 0))
  if (rev & (1 << idx)) ~= 0 then
    return -1
  end
  return 1
end

local function be_i16(hi, lo)
  local value = ((hi & 0xFF) << 8) | (lo & 0xFF)
  if value >= 32768 then
    value = value - 65536
  end
  return value
end

local function constrain(value, lo, hi)
  if value > hi then
    return hi
  elseif value < lo then
    return lo
  end
  return value
end

local function command_norm(idx)
  if script_fresh and as_number(millis() - script_ms) < 500 then
    return script_norm[idx + 1] or 0
  end
  return 0
end

local function healthy(idx)
  local fb = esc[idx + 1]
  if not fb.detected or fb.last_ms == nil then
    return false
  end
  return as_number(millis() - fb.last_ms) < FEEDBACK_TIMEOUT_MS
end

local function apply_switch()
  local want_can = pget("CAN_ENABLE", 0) >= 0.5
  if want_can == latched_can then
    return
  end
  if arming:is_armed() then
    local now = millis()
    if as_number(now - last_switch_warn) >= 5000 then
      last_switch_warn = now
      gcs:send_text(MAV_SEVERITY.WARNING, "DJICAN: disarm to switch CAN_ENABLE")
    end
    return
  end
  latched_can = want_can
  for i = 1, MAX_ESC do
    esc[i].integrator = 0
    esc[i].last_error = 0
    esc[i].current_a = 0
  end
  if latched_can then
    gcs:send_text(MAV_SEVERITY.INFO, "DJICAN: CAN ON")
  else
    gcs:send_text(MAV_SEVERITY.INFO, "DJICAN: CAN OFF")
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
  id = id & 0x7FF
  if id <= FB_BASE or id > FB_BASE + MAX_ESC then
    return
  end
  local idx = id - FB_BASE - 1
  local fb = esc[idx + 1]
  local angle = (((frame:data(0) & 0xFF) << 8) | (frame:data(1) & 0xFF)) % COUNTS_PER_REV
  if fb.have_angle then
    local delta = angle - fb.angle
    if delta > COUNTS_PER_REV / 2 then
      delta = delta - COUNTS_PER_REV
    elseif delta < -COUNTS_PER_REV / 2 then
      delta = delta + COUNTS_PER_REV
    end
    fb.rotor_counts = fb.rotor_counts + delta
  end
  fb.angle = angle
  fb.have_angle = true
  fb.rotor_rpm = be_i16(frame:data(2), frame:data(3))
  fb.current_raw = be_i16(frame:data(4), frame:data(5))
  fb.temperature = frame:data(6) & 0xFF
  fb.last_ms = millis()
  if not fb.detected then
    fb.detected = true
    gcs:send_text(MAV_SEVERITY.INFO, string.format("DJICAN: found ESC %d (SERVO%d)", idx + 1, idx + 1))
  end
end

local function drain()
  -- Four C620s send about 4000 frames/s. Reading until the buffer stays
  -- empty never returns and the scripting VM reports exceeded time.
  rx_frames = 0
  for _ = 1, MAX_FRAMES do
    local frame = driver:read_frame()
    if not frame then
      return
    end
    rx_frames = rx_frames + 1
    accept_frame(frame)
  end
end

local function put_i16(frame, index, value)
  value = math.floor(constrain(value, -RAW_MAX, RAW_MAX))
  if value < 0 then
    value = value + 65536
  end
  frame:data(index, (value >> 8) & 0xFF)
  frame:data(index + 1, value & 0xFF)
end

local function send_group(std_id, raw)
  local frame = CANFrame()
  frame:id(std_id)
  frame:dlc(8)
  for i = 0, 3 do
    put_i16(frame, i * 2, raw[i + 1])
  end
  driver:write_frame(frame, 1000)
end

-- Little-endian samples: CSR2 uses 11 bytes with uint32 time; CRP2 adds target RPM.
local RPM_HOLD_MS = 5000
local RPM_SAMPLE_MS = 50
local CURRENT_SAMPLE_MS = 25
local step
local pending_result
local was_armed = false
local inhibit_drive = false
local tune
local tune_last_loop
local tune_loop_ms = 0
local rx_age_before = {}
local TUNE_VALIDATE_START_TIMEOUT_MS = 10000
local START_TIMEOUT_S = 2
local START_FADE_S = 0.3
local START_MOTION_S = 0.2

local update_tune, save_tune_results, record_tune_diag
do
local tune_was_armed = false
local tune_result
local tune_log
local tune_diag
local TUNE_LOG_BLOCK_BYTES = 1024
local TUNE_LOG_SAMPLE_BYTES = 24 * 1024
local TUNE_LOG_SAMPLE_MS = 200
local TUNE_START_A = 1.8
local TUNE_INCREMENT_A = 0.05
local TUNE_MAX_SAMPLES = 4096
local TUNE_SAMPLE_MS = 50
local TUNE_MAX_DELAY = 10
local TUNE_RPM_BLOCK_SAMPLES = 64

local function tune_message(level, message)
  gcs:send_text(level, "CAN SIMC: " .. message)
end

record_tune_diag = function(reason)
  if not tune_diag or not tune then
    return
  end
  if #tune_diag >= 32 then
    table.remove(tune_diag, 1)
  end
  local now = millis()
  local function age(index)
    local last = esc[index].last_ms
    return last and as_number(now - last) or -1
  end
  tune_diag[#tune_diag + 1] = string.format(
    "%.0f,%d,%s,%.2f,%.2f,%.2f,%d,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%s\n",
    as_number(now - tune.start), tune.index, tune.phase or "start", tune_loop_ms,
    0, 0, rx_frames, rx_age_before[1] or -1, rx_age_before[2] or -1,
    rx_age_before[3] or -1, rx_age_before[4] or -1, age(1), age(2), age(3), age(4), reason)
end

-- CSI1 record 4: validation outcome, lambda scale, gains, start time,
-- peak RPM, tail MAE and time saturated. Missing measurements are -1.
local tune_write
local function log_validation(status)
  if tune.phase ~= "validate" then
    return
  end
  local now = millis()
  tune_write(string.pack("<BBbI4BBffffff", 4, tune.index, tune.sign, as_number(now - tune.start),
    status, tune.lambda_scale, tune.P, tune.I,
    tune.tracking_since and as_number(tune.tracking_since - tune.since) * 0.001 or -1,
    tune.peak, tune.tail_n > 0 and tune.tail_sum / tune.tail_n or -1, tune.saturated_s))
end

local function stop_tune(reason)
  log_validation(2)
  record_tune_diag(reason)
  tune_message(MAV_SEVERITY.WARNING, "ESC " .. tune.index .. " " .. reason)
  tune = nil
end

-- Keep the CSI1 format. Only decimate the diagnostic trace, not the model input.
tune_write = function(record)
  if record:byte() == 1 then
    local now = millis()
    if tune_log.last_sample and as_number(now - tune_log.last_sample) < TUNE_LOG_SAMPLE_MS then
      return true
    end
    if tune_log.bytes + #record > TUNE_LOG_SAMPLE_BYTES then
      if not tune_log.limited then
        tune_log.limited = true
        record_tune_diag("sample log capacity reached")
      end
      return true
    end
    tune_log.last_sample = now
  end
  local blocks = tune_log.blocks
  local index = #blocks
  local block = blocks[index]
  if #block + #record > TUNE_LOG_BLOCK_BYTES then
    blocks[index + 1] = record
  else
    blocks[index] = block .. record
  end
  tune_log.bytes = tune_log.bytes + #record
  return true
end

-- Fit an affine ARX model by centering the regressors. The intercept accounts
-- for the torque required to keep the wheel moving above the dead zone.
local function identification_sample(blocks, index)
  local block = blocks[(index - 1) // TUNE_RPM_BLOCK_SAMPLES + 1]
  return string.unpack("<fi2", block, 6 * ((index - 1) % TUNE_RPM_BLOCK_SAMPLES) + 1)
end

local function advance_speed_fit(fit)
  local first = TUNE_MAX_DELAY + 1
  local last = math.min(fit.next_sample + 31, fit.count - 1)
  local n, sx, su, sy = fit.n, fit.sx, fit.su, fit.sy
  local xx, uu, yy, xu, xy, uy = fit.xx, fit.uu, fit.yy, fit.xu, fit.xy, fit.uy
  -- Use the same output samples for every candidate delay.
  for k = fit.next_sample, last do
    local x = identification_sample(fit.blocks, k)
    local y = identification_sample(fit.blocks, k + 1)
    local _, ma = identification_sample(fit.blocks, k - fit.delay)
    local u = ma / 1000
    n, sx, su, sy = n + 1, sx + x, su + u, sy + y
    xx, uu, yy = xx + x*x, uu + u*u, yy + y*y
    xu, xy, uy = xu + x*u, xy + x*y, uy + u*y
  end
  if last < fit.count - 1 then
    fit.n, fit.sx, fit.su, fit.sy = n, sx, su, sy
    fit.xx, fit.uu, fit.yy, fit.xu, fit.xy, fit.uy = xx, uu, yy, xu, xy, uy
    fit.next_sample = last + 1
    return false
  end
  xx, uu, yy = xx - sx*sx/n, uu - su*su/n, yy - sy*sy/n
  xu, xy, uy = xu - sx*su/n, xy - sx*sy/n, uy - su*sy/n
  local det = xx*uu - xu*xu
  if det > 1e-9 and yy > 1e-9 then
    local a = (xy*uu - uy*xu) / det
    local b = (uy*xx - xy*xu) / det
    local r2 = (a*xy + b*uy) / yy
    if a > 0 and a < 1 and b > 0 and (not fit.best or r2 > fit.best.r2) then
      local ts = TUNE_SAMPLE_MS / 1000
      fit.best = { K = b / (1-a), tau = -ts / math.log(a), L = fit.delay*ts, r2 = r2 }
    end
  end
  fit.delay, fit.next_sample = fit.delay + 1, first
  fit.n, fit.sx, fit.su, fit.sy = 0, 0, 0, 0
  fit.xx, fit.uu, fit.yy, fit.xu, fit.xy, fit.uy = 0, 0, 0, 0, 0, 0
  return fit.delay > TUNE_MAX_DELAY, fit.best
end

local function simc_gains(model, lambda)
  local span = lambda + model.L
  local kp = model.tau / (model.K * span)
  return kp, kp / math.min(model.tau, 4 * span)
end

local function tune_rest(now, phase)
  tune.phase, tune.since, tune.rest_since = phase, now, nil
  tune.current, tune.reference, tune.integral = 0, 0, 0
  tune.last_error = nil
end

save_tune_results = function()
  if arming:is_armed() then
    return
  end
  if tune_log then
    local file = io.open("APM/CANSIMC.BIN", "wb")
    local ok
    if file then
      for _, block in ipairs(tune_log.blocks) do
        ok = file:write(block)
        if not ok then break end
      end
      local closed = file:close()
      ok = ok and closed
    end
    if ok and tune_log.limited then
      tune_message(MAV_SEVERITY.WARNING, "log sample capacity reached")
    end
    tune_log = nil
    tune_message(ok and MAV_SEVERITY.NOTICE or MAV_SEVERITY.WARNING,
      ok and "APM/CANSIMC.BIN saved" or "log save failed")
    if not ok then
      tune_result = nil
    end
  end
  if tune_diag then
    local file = io.open("APM/CANSDIAG.CSV", "wb")
    local ok
    if file then
      ok = file:write("ms,esc,phase,loop_ms,last_sd_ms,peak_sd_ms,rx_frames,before1,before2,before3,before4,after1,after2,after3,after4,event\n")
      for _, row in ipairs(tune_diag) do
        if not ok then break end
        ok = file:write(row)
      end
      local closed = file:close()
      ok = ok and closed
    end
    tune_diag = nil
    tune_message(ok and MAV_SEVERITY.NOTICE or MAV_SEVERITY.WARNING,
      ok and "APM/CANSDIAG.CSV saved" or "diagnostic save failed")
  end
  if tune_result then
    local saved = 0
    for index, gain in ipairs(tune_result) do
      local p_saved = param:set_and_save("CAN_P_" .. index, gain.P)
      local i_saved = param:set_and_save("CAN_I_" .. index, gain.I)
      local forward_saved = param:set_and_save("CAN_START_F_" .. index, gain.forward)
      local reverse_saved = param:set_and_save("CAN_START_R_" .. index, gain.reverse)
      if p_saved and i_saved and forward_saved and reverse_saved then
        saved = saved + 1
      else
        tune_message(MAV_SEVERITY.WARNING, "gain save failed ESC " .. index)
      end
    end
    tune_result = nil
    if pget("SCR_USER1", 0) == 4 and not param:set_and_save("SCR_USER1", 0) then
      tune_message(MAV_SEVERITY.WARNING, "reset SCR_USER1 manually")
    end
    tune_message(MAV_SEVERITY.NOTICE, string.format("saved PI for %d ESCs", saved))
  end

end

local function begin_speed_fit()
  if not tune.dead or tune.rpm_count <= TUNE_MAX_DELAY + 1 then
    stop_tune("insufficient staircase data")
    return
  end
  tune.fit = { blocks = tune.rpm_blocks, count = tune.rpm_count, delay = 0,
    next_sample = TUNE_MAX_DELAY + 1, n = 0, sx = 0, su = 0, sy = 0,
    xx = 0, uu = 0, yy = 0, xu = 0, xy = 0, uy = 0 }
  tune.phase, tune.current = "fit", 0
  tune_message(MAV_SEVERITY.NOTICE, "ESC 1 fitting model")
end

update_tune = function(armed, enabled, max_rpm, max_current, imax)
  local now = millis()
  local selected = pget("SCR_USER1", 0) == 4
  if not armed and tune then
    stop_tune("disarmed")
  end

  local start_current = TUNE_START_A
  local blocked
  if not selected then
    blocked = "SCR_USER1 changed"
  elseif pget("CAN_ENABLE", 0) ~= 1 then
    blocked = "CAN_ENABLE must be 1"
  elseif not enabled then
    blocked = "outputs inhibited"
  elseif vehicle:get_mode() ~= 0 then
    blocked = "MANUAL required"
  elseif max_rpm <= 44 then
    blocked = string.format("RPM cap %.1f <= 44", max_rpm)
  elseif max_current < start_current then
    blocked = string.format("CAN_CUR %.2f < start %.2f A", max_current, start_current)
  elseif imax <= 0 then
    blocked = "CAN_IMAX <= 0 A"
  elseif pget("CAN_STEP_RESP", 0) == 1 or pget("CAN_STEP_RPM", 0) == 1 then
    blocked = "another CAN test selected"
  end
  if armed and not tune_was_armed and selected then
    inhibit_drive = true
    local step_ms = pget("CAN_TUNE_TIME", DEFAULT_TUNE_STEP_TIME_S) * 1000
    if not blocked and step_ms >= 1000 and step_ms <= 10000 then
      tune_log = { blocks = { "CSI1" }, bytes = 4 }
      tune = { index = 1, sign = 1, start = now, step_ms = step_ms,
        results = {}, models = {} }
      tune_diag = {}
      tune_rest(now, "rest_search")
    else
      tune_message(MAV_SEVERITY.WARNING, blocked or "check CAN_TUNE_TIME (1-10s)")
    end
  end
  tune_was_armed = armed
  if not tune then
    return
  end
  if blocked then
    stop_tune(blocked)
    return
  end
  local gear = math.max(pget("CAN_GEAR", 19.2032), 1)
  local phase = tune.phase
  local fb = esc[1]
  if not healthy(0) then
    local last = fb.last_ms
    stop_tune(last and string.format("FB ESC 1 %.0f/%d ms", as_number(now - last), FEEDBACK_TIMEOUT_MS)
      or "ESC 1 no feedback")
    return
  end
  local rpm = tune.sign * direction(0) * fb.rotor_rpm / gear
  local speed_limit = max_rpm
  if phase == "search" and rpm >= speed_limit then
    tune_message(MAV_SEVERITY.NOTICE, string.format("ESC 1 limit %.1f RPM at %+.2f A",
      rpm, tune.sign * tune.current))
    begin_speed_fit()
    return
  elseif (phase == "validate" and math.abs(rpm) >= speed_limit)
      or (phase == "search" and rpm <= -speed_limit) then
    stop_tune(string.format("RPM limit %.1f/%.1f", math.abs(rpm), speed_limit))
    return
  end

  local elapsed = as_number(now - tune.since)
  if phase == "rest_search" or phase == "rest_validate" then
    if math.abs(rpm) > 1 then
      tune.rest_since = nil
    else
      tune.rest_since = tune.rest_since or now
    end
    if elapsed >= 15000 then
      stop_tune(string.format("rest 15s: %.1f RPM (<=1)", math.abs(rpm)))
    elseif tune.rest_since and as_number(now - tune.rest_since) >= 1000 then
      tune.since, tune.last_sample, tune.next_sample = now, now, now + uint32_t(TUNE_SAMPLE_MS)
      if phase == "rest_search" then
        tune.phase, tune.motion_since, tune.dead = "search", nil, nil
        tune.current = start_current
        tune.rpm_blocks, tune.rpm_count = {}, 0
        tune_message(MAV_SEVERITY.NOTICE, string.format("ESC %d search: %+.3f A",
          tune.index, tune.sign * tune.current))
      else
        tune.phase, tune.peak, tune.tail_sum, tune.tail_n = "validate", 0, 0, 0
        tune.motion_since, tune.tracking_since, tune.saturated_s = nil, nil, 0
        tune.reference = 40
        tune_message(MAV_SEVERITY.NOTICE, string.format("ESC %d validate %+d RPM", tune.index,
          math.floor(tune.sign * tune.reference)))
      end
    end
    return
  end

  if phase == "fit" then
    local done, model = advance_speed_fit(tune.fit)
    if not done then
      return
    end
    tune.fit = nil
    tune.rpm_blocks, tune.rpm_count = nil, nil
    if not model or model.r2 < 0.9 then
      if tune.retried then
        stop_tune(model and string.format("R2 %.3f < 0.900", model.r2) or "no valid ARX model")
      else
        tune.retried = true
        tune_rest(now, "rest_search")
        tune_message(MAV_SEVERITY.WARNING, "poor fit; repeating ESC " .. tune.index)
      end
      return
    end
    if not tune_write(string.pack("<BBbI4fffff", 2, tune.index, tune.sign,
        as_number(now - tune.start), tune.dead, model.K, model.tau, model.L, model.r2)) then
      return
    end
    tune.timing_retried = nil
    model.dead = tune.dead
    tune.models[#tune.models + 1] = model
    if tune.sign == 1 then
      tune.sign, tune.retried = -1, nil
      tune_rest(now, "rest_search")
    else
      tune.lambda_scale = 1
      tune.P, tune.I = 0, 0
      for _, fit in ipairs(tune.models) do
        local kp, ki = simc_gains(fit, 3 * math.max(fit.L, TUNE_SAMPLE_MS / 1000))
        tune.P, tune.I = tune.P + kp / #tune.models, tune.I + ki / #tune.models
      end
      tune.sign = 1
      tune_rest(now, "rest_validate")
    end
    return
  end

  local sample_ms = as_number(now - tune.last_sample)
  if (now - tune.next_sample):toint() < 0 then
    return
  end
  tune.last_sample = now
  tune.next_sample = tune.next_sample + uint32_t(TUNE_SAMPLE_MS)
  if (now - tune.next_sample):toint() >= 0 then
    tune.next_sample = now + uint32_t(TUNE_SAMPLE_MS)
  end
  if (phase == "search" or phase == "validate") and sample_ms > 2 * TUNE_SAMPLE_MS then
    local reason = string.format("%s gap %.0f/%d ms", phase, sample_ms, 2 * TUNE_SAMPLE_MS)
    if tune.timing_retried == phase then
      stop_tune(reason)
    else
      log_validation(3)
      record_tune_diag(reason .. " retry")
      tune.timing_retried = phase
      tune.rpm_blocks, tune.rpm_count = nil, nil
      tune_rest(now, phase == "search" and "rest_search" or "rest_validate")
      tune_message(MAV_SEVERITY.WARNING, string.format("ESC %d %s; retry", tune.index, reason))
    end
    return
  end
  local next_current = tune.current
  if phase == "search" then
    if rpm >= 3 then
      tune.motion_since = tune.motion_since or now
      if not tune.dead and as_number(now - tune.motion_since) >= 200 then
        tune.dead = tune.current
        tune_message(MAV_SEVERITY.NOTICE, string.format("ESC %d breakaway %+.3f A",
          tune.index, tune.sign * tune.dead))
      end
    else
      tune.motion_since = nil
    end
    local n = tune.rpm_count
    if n >= TUNE_MAX_SAMPLES then
      stop_tune("staircase sample capacity")
      return
    end
    if elapsed >= tune.step_ms then
      if tune.current >= max_current then
        stop_tune(string.format("CAN_CUR %.2f A at %.1f/%.1f RPM", max_current, rpm, speed_limit))
        return
      end
      next_current = math.min(max_current, tune.current + TUNE_INCREMENT_A)
      tune.since = now
      if not tune.dead then
        tune.motion_since = nil
      end
      tune_message(MAV_SEVERITY.NOTICE, string.format("ESC 1 search: %+.3f A", tune.sign * next_current))
    end
    local block_index = n // TUNE_RPM_BLOCK_SAMPLES + 1
    local blocks = tune.rpm_blocks
    blocks[block_index] = (blocks[block_index] or "") .. string.pack("<fi2", rpm,
      math.floor(next_current * 1000 + 0.5))
    tune.rpm_count = n + 1
  else
    local error_rpm = tune.reference - rpm
    tune.peak = math.max(tune.peak, rpm)
    if tune.peak > 1.1 * tune.reference then
      if tune.lambda_scale >= 8 then
        stop_tune(string.format("peak %.1f/%.1f RPM", tune.peak, 1.1 * tune.reference))
        return
      end
      log_validation(1)
      tune.lambda_scale = tune.lambda_scale * 2
      tune.P, tune.I = 0, 0
      for _, fit in ipairs(tune.models) do
        local kp, ki = simc_gains(fit, tune.lambda_scale * 3 * math.max(fit.L, TUNE_SAMPLE_MS / 1000))
        tune.P, tune.I = tune.P + kp / #tune.models, tune.I + ki / #tune.models
      end
      tune.sign = 1
      tune_rest(now, "rest_validate")
      return
    end
    if not tune.tracking_since then
      if rpm >= 3 then
        tune.motion_since = tune.motion_since or now
      else
        tune.motion_since = nil
      end
      if tune.motion_since and as_number(now - tune.motion_since) >= 200 then
        tune.tracking_since = now
        tune_message(MAV_SEVERITY.NOTICE, string.format("ESC %d moving; track for 8s", tune.index))
      elseif elapsed >= TUNE_VALIDATE_START_TIMEOUT_MS then
        stop_tune(string.format("start 10s: %.1f/3 RPM", rpm))
        return
      end
    end
    local tracking_ms = tune.tracking_since and as_number(now - tune.tracking_since) or 0
    if tracking_ms >= 6000 then
      tune.tail_sum, tune.tail_n = tune.tail_sum + math.abs(error_rpm), tune.tail_n + 1
    end
    if tracking_ms >= 8000 then
      if tune.tail_n < 10 or tune.tail_sum / tune.tail_n > 0.1 * tune.reference then
        stop_tune(tune.tail_n < 10 and string.format("samples %d/10", tune.tail_n)
          or string.format("MAE %.1f/%.1f RPM", tune.tail_sum / tune.tail_n, 0.1 * tune.reference))
        return
      end
      log_validation(0)
      if tune.sign == 1 then
        tune.sign = -1
        tune.timing_retried = nil
        tune_rest(now, "rest_validate")
        return
      end
      if not tune_write(string.pack("<BBI4ff", 3, tune.index, as_number(now - tune.start), tune.P, tune.I)) then
        return
      end
      tune.results[tune.index] = { P = tune.P, I = tune.I, forward = tune.models[1].dead, reverse = tune.models[2].dead }
      tune_message(MAV_SEVERITY.NOTICE, string.format("ESC %d P %.4f I %.4f", tune.index, tune.P, tune.I))
      tune_result, tune = tune.results, nil
      tune_message(MAV_SEVERITY.NOTICE, "complete; disarm to save")
      return
    end
  end
  tune.current = next_current
  local command = phase == "validate" and esc[tune.index].current_a or tune.current * tune.sign
  tune_write(string.pack("<BBbI4ffff", 1, tune.index, tune.sign, as_number(now - tune.start),
    tune.reference * tune.sign, rpm * tune.sign, command,
    direction(tune.index - 1) * esc[tune.index].current_raw / RAW_PER_AMP))
end

end

local function save_step()
  local result = pending_result
  pending_result = nil
  local path = result.rpm and "APM/CANRPM.BIN" or "APM/CANSTEP.BIN"
  local file = io.open(path, "wb")
  local ok
  if file then
    local written = file:write(result.rpm and "CRP2" or "CSR2")
    if written then
      -- Avoid allocating a second buffer as large as the entire result.
      for _, sample in ipairs(result.samples) do
        written = file:write(sample)
        if not written then
          break
        end
      end
    end
    local closed = file:close()
    ok = written and closed
  end
  gcs:send_text(ok and MAV_SEVERITY.INFO or MAV_SEVERITY.WARNING,
    "CAN step: " .. result.reason .. (ok and " (" .. path .. ")" or " (save failed)"))
end

local function finish_step(reason)
  step.reason = reason
  pending_result = step
  step = nil
  gcs:send_text(MAV_SEVERITY.INFO, "CAN step: " .. reason .. "; disarm to save")
end

local function set_current_plateau(now, level)
  step.level = level
  step.current = math.min(step.max_current, step.current_min + level * step.current_inc)
  step.phase_start = now
  gcs:send_text(MAV_SEVERITY.NOTICE, string.format("CAN ESC %d: step %d, %.3f A",
    step.index, level + 1, step.current))
end

local function update_step(armed, enabled, max_rpm, max_current)
  local now = millis()
  local rpm_requested = pget("CAN_STEP_RPM", 0) == 1
  local current_requested = pget("CAN_STEP_RESP", 0) == 1
  if not armed then
    inhibit_drive = false
  elseif not was_armed and pget("SCR_USER1", 0) ~= 4 and latched_can and vehicle:get_mode() == 0
      and (rpm_requested or current_requested) then
    inhibit_drive = true
    if rpm_requested and current_requested then
      gcs:send_text(MAV_SEVERITY.WARNING, "CAN step: select only one test and rearm")
      was_armed = true
      return
    end
    local current_min = pget("CAN_STEP_MIN", DEFAULT_STEP_MIN)
    local current_inc = pget("CAN_STEP_INC", DEFAULT_STEP_INC)
    local hold_ms = pget("CAN_STEP_TIME", DEFAULT_STEP_TIME) * 1000
    local first_esc, last_esc, esc_increment = 1, MAX_ESC, 1
    local levels
    if current_requested then
      local selection = math.tointeger(pget("CAN_RESP_ESC", 6))
      if not selection or selection < 1 or selection > 6 then
        gcs:send_text(MAV_SEVERITY.WARNING, "CAN step: CAN_RESP_ESC must be 1-6")
        was_armed = true
        return
      end
      if selection <= MAX_ESC then
        first_esc, last_esc = selection, selection
      elseif selection == 6 then
        first_esc, last_esc, esc_increment = MAX_ESC, 1, -1
      end
      if not (current_min > 0 and current_min <= max_current and current_inc > 0
          and current_inc <= 20 and hold_ms >= LOOP_MS) then
        gcs:send_text(MAV_SEVERITY.WARNING, "CAN step: invalid MIN, INC or TIME")
        was_armed = true
        return
      end
      levels = 1 + math.ceil((max_current - current_min) / current_inc)
      if not (levels * MAX_ESC * (hold_ms + LOOP_MS) <= 0x7fffffff) then
        gcs:send_text(MAV_SEVERITY.WARNING, "CAN step: test duration too long")
        was_armed = true
        return
      end
    end
    step = { index = first_esc, last_esc = last_esc, esc_increment = esc_increment,
      start = now, phase_start = now, samples = {},
      rpm = rpm_requested, param = rpm_requested and "CAN_STEP_RPM" or "CAN_STEP_RESP",
      current_min = current_min, current_inc = current_inc, hold_ms = hold_ms, max_current = max_current }
    if current_requested then
      -- Bound the buffered samples across all plateaus of each ESC.
      step.sample_ms = math.max(CURRENT_SAMPLE_MS, math.ceil(levels * (hold_ms + LOOP_MS) / 400))
    end
    gcs:send_text(MAV_SEVERITY.INFO, string.format("CAN step: starting ESC %d", step.index))
  end
  was_armed = armed
  if not step then
    return
  end
  local fb = esc[step.index]
  local rotor_to_wheel_rpm = fb.rotor_rpm / math.max(pget("CAN_GEAR", 19.2032), 1)
  local wheel_speed = math.abs(rotor_to_wheel_rpm)
  local abort_reason
  if not armed then
    abort_reason = "disarmed"
  elseif not enabled then
    abort_reason = "CAN outputs inhibited"
  elseif vehicle:get_mode() ~= 0 then
    abort_reason = "left MANUAL"
  elseif pget(step.param, 0) ~= 1 then
    abort_reason = "test selector changed"
  elseif not healthy(step.index - 1) then
    abort_reason = "feedback missing or stale"
  elseif max_rpm < 15 or max_current <= 0 then
    abort_reason = "RPM or current limit too low"
  elseif not step.rpm and step.max_current > max_current then
    abort_reason = "current limit reduced"
  elseif step.rpm and wheel_speed >= max_rpm then
    abort_reason = string.format("RPM limit %.1f/%.1f", wheel_speed, max_rpm)
  end
  if abort_reason then
    gcs:send_text(MAV_SEVERITY.WARNING, string.format("CAN step ESC %d: %s", step.index, abort_reason))
    finish_step("aborted")
    return
  end

  local elapsed = as_number(now - step.phase_start)
  if step.rpm and not step.base then
    local target = math.min(20, max_rpm * 0.5)
    if math.abs(fb.wheel_rpm - target) > target * 0.1 then
      step.stable_since = nil
    else
      step.stable_since = step.stable_since or now
      if as_number(now - step.stable_since) >= 400 then
        step.base = fb.current_a
        step.phase_start = now
        step.stable_since = nil
      end
    end
    if not step.base and elapsed >= 3000 then
      finish_step("could not stabilize")
    end
    return
  end

  if step.rpm then
    if elapsed < 3 * RPM_HOLD_MS then
      return
    end
  elseif wheel_speed >= max_rpm then
    gcs:send_text(MAV_SEVERITY.INFO, string.format("CAN step ESC %d: RPM limit %.1f/%.1f",
      step.index, wheel_speed, max_rpm))
  elseif not step.level then
    set_current_plateau(now, 0)
    return
  elseif elapsed < step.hold_ms then
    return
  elseif step.current < step.max_current then
    set_current_plateau(now, step.level + 1)
    return
  else
    gcs:send_text(MAV_SEVERITY.INFO, string.format("CAN step ESC %d: CAN_CUR reached", step.index))
  end

  if step.index == step.last_esc then
    if not param:set_and_save(step.param, 0) then
      gcs:send_text(MAV_SEVERITY.WARNING, "CAN step: could not reset " .. step.param)
    end
    finish_step("complete")
    return
  end
  step.index = step.index + step.esc_increment
  step.phase_start = now
  step.base, step.stable_since = nil, nil
  step.level, step.current = nil, nil
  step.last_sample = nil
end

local function speed_current(fb, reference, kp, ki, kd, kff, dt, imax, limit, breakaway, start_timeout)
  local err = reference - fb.wheel_rpm
  local sign = reference > 0 and 1 or -1
  local boost = 0
  if reference == 0 or sign ~= fb.start_sign then
    fb.start_sign = reference ~= 0 and sign or nil
    fb.start_time = reference ~= 0 and math.abs(fb.wheel_rpm) <= 1 and 0 or nil
    fb.start_motion = nil
  end
  if fb.start_time then
    fb.start_time = fb.start_time + dt
    if sign * fb.wheel_rpm >= 3 then
      fb.start_motion = (fb.start_motion or 0) + dt
    else
      fb.start_motion = nil
    end
    local fade = math.max(0, (fb.start_motion or 0) - START_MOTION_S)
    local weight = 1 - fade / START_FADE_S
    if weight <= 0 or fb.start_time >= start_timeout then
      fb.start_time = nil
    else
      local base = kp * err + kff * reference + fb.integrator
      boost = sign * weight * math.max(0, math.min(breakaway, limit) - sign * base)
    end
  end
  local derivative = dt > 0 and kd * (err - fb.last_error) / dt or 0
  fb.last_error = err
  local correction = kff * reference + kp * err + derivative + boost
  local output = correction + fb.integrator
  if not ((output >= limit and err > 0) or (output <= -limit and err < 0)) then
    fb.integrator = constrain(fb.integrator + ki * err * dt, -imax, imax)
  end
  output = correction + fb.integrator
  return constrain(output, -limit, limit), math.abs(output) >= limit
end

local function run_control()
  local now_us = micros()
  local dt = 0.01
  if last_us ~= nil then
    dt = constrain(as_number(now_us - last_us) * 1.0e-6, 0, 0.05)
  end
  last_us = now_us

  local gear = math.max(pget("CAN_GEAR", 19.2032), 1)
  local max_rpm = FULL_SCALE_RPM * constrain(pget("THRO_MAX_LUA", 90), 0, 100) / 100
  local max_current = constrain(pget("CAN_CUR", 10), 0, 20)
  local imax = constrain(pget("CAN_IMAX", 10), 0, max_current)
  local kff = pget("CAN_FF", 0)
  local hold_disarmed = (math.floor(pget("CAN_OPT", 1)) & 2) ~= 0
  local armed = arming:is_armed()
  local safety_live = not SRV_Channels:get_safety_state()
  local estop = SRV_Channels:get_emergency_stop()
  local enabled = latched_can and safety_live and not estop and (armed or hold_disarmed)

  update_step(armed, enabled, max_rpm, max_current)
  update_tune(armed, enabled, max_rpm, max_current, imax)

  local raw = { 0, 0, 0, 0 }
  local detected = 0
  for i = 0, MAX_ESC - 1 do
    local fb = esc[i + 1]
    local dir = direction(i)
    fb.wheel_rpm = dir * fb.rotor_rpm / gear
    if fb.detected then
      detected = detected | (1 << i)
    end
    local active = enabled and fb.have_angle and healthy(i)
      and (not inhibit_drive or (step and step.index == i + 1)
        or (tune and tune.index == i + 1))
    if not active then
      fb.integrator = 0
      fb.last_error = 0
      fb.start_sign, fb.start_time, fb.start_motion = nil, nil, nil
      fb.target_rpm = 0
      fb.current_a = 0
    else
      local current_a
      if (step and not step.rpm) or (tune and tune.phase ~= "validate") then
        -- Identification and validation own the active ESC current.
        fb.integrator = 0
        fb.last_error = 0
        fb.start_sign, fb.start_time, fb.start_motion = nil, nil, nil
        fb.target_rpm = 0
        current_a = tune and constrain(tune.sign * tune.current, -max_current, max_current) or (step.current or 0)
      else
        local cmd = 0
        if step then
          local target = math.min(20, max_rpm * 0.5)
          local elapsed = as_number(millis() - step.phase_start)
          if step.base and elapsed >= RPM_HOLD_MS and elapsed < 2 * RPM_HOLD_MS then
            target = target * 1.5
          end
          cmd = target / max_rpm
        elseif armed then
          cmd = command_norm(i)
        end
        local esc_number = tostring(i + 1)
        fb.target_rpm = tune and tune.sign * tune.reference or cmd * max_rpm
        local kp = tune and tune.P or pget("CAN_P_" .. esc_number, 0.03)
        local ki = tune and tune.I or pget("CAN_I_" .. esc_number, 0.3)
        local kd = pget("CAN_D_" .. esc_number, 0)
        local start_param = fb.target_rpm < 0 and "CAN_START_R_" or "CAN_START_F_"
        local breakaway = tune and tune.models[tune.sign == 1 and 1 or 2].dead
          or pget(start_param .. esc_number, 0)
        local saturated
        local start_timeout = tune and TUNE_VALIDATE_START_TIMEOUT_MS / 1000 or START_TIMEOUT_S
        current_a, saturated = speed_current(fb, fb.target_rpm, kp, ki, kd, kff, dt, imax, max_current,
          breakaway, start_timeout)
        if tune and saturated then
          tune.saturated_s = tune.saturated_s + dt
        end
      end
      if step and (step.rpm and step.base or step.current) then
        local now = millis()
        if not step.last_sample or as_number(now - step.last_sample) >= (step.rpm and RPM_SAMPLE_MS or step.sample_ms) then
          step.last_sample = now
          local elapsed_ms = as_number(now - step.start)
          local rpm10 = math.floor(fb.wheel_rpm * 10)
          local cmd_ma = math.floor(current_a * 1000)
          local meas_ma = math.floor(constrain(dir * fb.current_raw / RAW_PER_AMP * 1000, -32768, 32767))
          local sample
          if step.rpm then
            sample = string.pack("<BI4i2i2i2i2", step.index, elapsed_ms, rpm10, cmd_ma, meas_ma,
              math.floor(fb.target_rpm * 10 + 0.5))
          else
            sample = string.pack("<BI4i2i2i2", step.index, elapsed_ms, rpm10, cmd_ma, meas_ma)
          end
          step.samples[#step.samples + 1] = sample
        end
      end
      fb.current_a = current_a
      raw[i + 1] = dir * current_a * RAW_PER_AMP
    end
  end

  -- PWM mode leaves the bus silent so the C620 can follow the servo pulse.
  if not latched_can then
    return
  end
  if detected ~= 0 then
    send_group(CMD_1_4, raw)
  end
end

local function hold_trim()
  -- MAIN OUT 1..4 are wired to the C620 PWM inputs. A pulse, even at trim,
  -- makes the ESC leave CAN. Zero width suppresses the pulse while CAN commands.
  for i = 0, MAX_ESC - 1 do
    SRV_Channels:set_output_pwm_chan_timeout(i, 0, 300)
  end
end

local function check_health()
  if not latched_can or not arming:is_armed() then
    lost_mask = 0
    return
  end
  local lost = 0
  for i = 0, MAX_ESC - 1 do
    if esc[i + 1].detected and not healthy(i) then
      lost = lost | (1 << i)
    end
  end
  local now = millis()
  if lost ~= 0 and (lost ~= lost_mask or as_number(now - last_lost_warn) >= 5000) then
    last_lost_warn = now
    for i = 0, MAX_ESC - 1 do
      if (lost & (1 << i)) ~= 0 then
        gcs:send_text(MAV_SEVERITY.CRITICAL, string.format("DJICAN: ESC %d lost, motor stopped", i + 1))
      end
    end
  end
  if lost == 0 and lost_mask ~= 0 then
    gcs:send_text(MAV_SEVERITY.INFO, "DJICAN: ESC feedback restored")
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
  local simc = pget("SCR_USER1", 0) == 4 and pget("CAN_ENABLE", 0) == 1 and vehicle:get_mode() == 0
  local required_escs = simc and 1 or MAX_ESC
  for i = 0, required_escs - 1 do
    if not esc[i + 1].detected then
      arming:set_aux_auth_failed(auth_id, string.format("ESC %d not found", i + 1))
      return
    end
    if not healthy(i) then
      arming:set_aux_auth_failed(auth_id, string.format("ESC %d no feedback", i + 1))
      return
    end
  end
  arming:set_aux_auth_passed(auth_id)
end

local function write_log()
  if inhibit_drive then
    return
  end
  if (math.floor(pget("CAN_OPT", 1)) & 1) == 0 then
    return
  end
  local now = millis()
  if as_number(now - last_log) < 20 then
    return
  end
  last_log = now
  local gear = math.max(pget("CAN_GEAR", 19.2032), 1)
  for i = 0, MAX_ESC - 1 do
    local fb = esc[i + 1]
    if fb.detected then
      local dir = direction(i)
      logger:write("DJIC", "Id,Tgt,Rpm,CurD,CurM,I,Pos,Temp", "BffffffB",
        i + 1,
        fb.target_rpm,
        fb.wheel_rpm,
        fb.current_a,
        dir * fb.current_raw / RAW_PER_AMP,
        fb.integrator,
        dir * fb.rotor_counts / (COUNTS_PER_REV * gear),
        fb.temperature)
    end
  end
end

local function send_telem()
  if esc_telem == nil or as_number(millis() - last_telem) < 20 then
    return
  end
  last_telem = millis()
  for i = 0, MAX_ESC - 1 do
    local fb = esc[i + 1]
    if fb.detected and healthy(i) then
      -- The Lua ESC telemetry binding takes an unsigned RPM. Control uses signed feedback.
      esc_telem:update_rpm(i, math.abs(fb.rotor_rpm), 0)
      local data = ESCTelemetryData()
      data:motor_temp_cdeg(math.floor(fb.temperature * 100))
      data:current(math.abs(fb.current_raw) / RAW_PER_AMP)
      esc_telem:update_telem_data(i, data, TELEM_MOTOR_TEMP | TELEM_CURRENT)
    end
  end
end

local motor_can = {}

function motor_can.init()
  ensure_params()
end

function motor_can.output_is_can()
  return latched_can and driver ~= nil
end

function motor_can.set_norms(norms)
  for i = 1, MAX_ESC do
    script_norm[i] = constrain(norms[i] or 0, -1, 1)
  end
  script_ms = millis()
  script_fresh = true
end

function motor_can.clear_script_norms()
  script_fresh = false
end

function motor_can.update()
  if not ensure_params() then
    return 1000
  end
  if can_api == nil then
    if not announced_driver then
      gcs:send_text(MAV_SEVERITY.WARNING, "DJICAN: firmware has no Lua CAN API")
      announced_driver = true
    end
    return 1000
  end
  if driver == nil then
    driver = can_api:get_device(25)
    if driver == nil then
      local now = millis()
      if as_number(now - last_driver_warn) >= 5000 then
        last_driver_warn = now
        gcs:send_text(MAV_SEVERITY.WARNING, "DJICAN: set CAN_Dx_PROTOCOL to Scripting (10)")
      end
      return 1000
    end
    if not announced_driver then
      announced_driver = true
      gcs:send_text(MAV_SEVERITY.INFO, "DJICAN: Lua CAN driver ready")
    end
    if auth_id == nil then
      auth_id = arming:get_aux_auth_id()
    end
  end

  apply_switch()
  if tune then
    local now = millis()
    tune_loop_ms = tune_last_loop and as_number(now - tune_last_loop) or 0
    for i = 1, MAX_ESC do
      local last = esc[i].last_ms
      rx_age_before[i] = last and as_number(now - last) or -1
    end
  end
  tune_last_loop = millis()
  drain()
  if tune and tune_loop_ms >= FEEDBACK_TIMEOUT_MS then
    record_tune_diag("control loop gap")
  end
  run_control()
  if latched_can then
    hold_trim()
    check_health()
  end
  prearm()
  send_telem()
  if pending_result and not arming:is_armed() then
    save_step()
  end
  write_log()
  save_tune_results()
  return LOOP_MS
end

return motor_can
