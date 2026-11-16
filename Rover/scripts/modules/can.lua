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
-- Keep the original CAN key when available so saved settings still load.
local PARAM_KEYS = { 19, 93, 94 }
local DEFAULT_STEP_MIN = 0.2
local DEFAULT_STEP_INC = 0.1
local DEFAULT_STEP_TIME = 5
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
    if param:add_table(key, "CAN_", 41) then
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
  spec[#spec + 1] = { 39, "TWD_ESC", 6 }
  spec[#spec + 1] = { 40, "TWD_EPO", 12 }
  spec[#spec + 1] = { 41, "TWD_G_RPM", 25 }
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
  for _ = 1, MAX_FRAMES do
    local frame = driver:read_frame()
    if not frame then
      return
    end
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
local START_TIMEOUT_S = 2
local START_FADE_S = 0.3
local START_MOTION_S = 0.2

local update_tune, save_tune_results, twiddle_selection, log_ground_sample
do
  local was_tune_armed = false
  local pending
  local TARGET, STEP, HOLD_MS = 125, 25, 2000
  local MAX_TRIALS = 40
  local MAX_TRACE_BYTES = 24 * 1024
  local TRACE_FORMAT = "<I4Bi2" .. string.rep("i2", 12)
  local MIN_STEP = { 0.0005, 0.001 }

  twiddle_selection = function()
    local value = math.tointeger(pget("CAN_TWD_ESC", 6))
    if value and value >= 1 and value <= 6 then
      return value
    end
  end

  local function notice(text)
    gcs:send_text(MAV_SEVERITY.NOTICE, "CAN Twiddle: " .. text)
  end

  local function stop(reason)
    pending.reason = reason
    tune = nil
    notice(reason .. "; disarm to save")
  end

  local function prepare_trial(now, gains)
    tune.trial = tune.trial + 1
    tune.P, tune.I = gains[1], gains[2]
    tune.gains = gains
    tune.reverse_pending = false
    tune.reference, tune.rest_since = nil, nil
    tune.since, tune.last = now, now
    tune.error_ms, tune.weight_ms = 0, 0
    if tune.ground then
      tune.error_by_esc = { 0, 0, 0, 0 }
      tune.trace_last = nil
      notice(string.format("ground T%d ESC %d %s", tune.trial, (tune.axis + 1) // 2,
        tune.trial == 1 and "baseline" or (tune.axis % 2 == 1 and "P" or "I")))
    else
      notice(string.format("ESC %d T%d P %.5f I %.5f", tune.index, tune.trial, tune.P, tune.I))
    end
  end

  local function prepare_esc(now)
    local p = math.max(0, pget("CAN_P_" .. tune.index, 0.03))
    local i = math.max(0, pget("CAN_I_" .. tune.index, 0.3))
    tune.best = { p, i }
    tune.dp = { math.max(0.001, p * 0.2), math.max(0.005, i * 0.2) }
    tune.axis, tune.direction, tune.trial = 1, 1, 0
    tune.best_score = math.huge
    prepare_trial(now, tune.best)
  end

  local function set_reference(now, reference)
    tune.reference, tune.since, tune.last = reference, now, now
    tune.trace_last = nil
    notice(string.format("%s T%d %+d RPM", tune.ground and "ground" or ("ESC " .. tune.index),
      tune.trial, reference))
  end

  local function complete_trial(now)
    local score = tune.error_ms / tune.weight_ms
    local improved = score < tune.best_score
    if improved then
      tune.best, tune.best_score = tune.gains, score
      if tune.ground then
        for index = 1, MAX_ESC do
          pending.best[index] = { tune.gains[2 * index - 1], tune.gains[2 * index] }
        end
      else
        pending.best[tune.index] = tune.best
      end
    end
    if tune.ground then
      for index = 1, MAX_ESC do
        pending.rows[#pending.rows + 1] = string.format("%d,%d,%.8g,%.8g,%.4f,%d,%.4f\n",
          index, tune.trial, tune.gains[2 * index - 1], tune.gains[2 * index],
          tune.error_by_esc[index] / tune.weight_ms, improved and 1 or 0, score)
      end
    else
      pending.rows[#pending.rows + 1] = string.format("%d,%d,%.8g,%.8g,%.4f,%d\n",
        tune.index, tune.trial, tune.P, tune.I, score, improved and 1 or 0)
    end
    notice(string.format("%s T%d MAE %.1f RPM", tune.ground and "ground" or ("ESC " .. tune.index), tune.trial, score))

    local axis = tune.axis
    if tune.trial > 1 then
      if improved then
        tune.dp[axis] = tune.dp[axis] * 1.1
        tune.axis, tune.direction = axis % #tune.dp + 1, 1
      elseif tune.direction == 1 and tune.best[axis] > 0 then
        tune.direction = -1
      else
        tune.dp[axis] = tune.dp[axis] * 0.9
        tune.axis, tune.direction = axis % #tune.dp + 1, 1
      end
    end
    local converged = true
    for index, delta in ipairs(tune.dp) do
      if delta > MIN_STEP[(index - 1) % 2 + 1] then
        converged = false
        break
      end
    end
    if tune.trial >= tune.max_trials or converged then
      if tune.index == tune.last_esc then
        stop("complete")
        return
      end
      tune.index = tune.index + tune.increment
      prepare_esc(now)
      return
    end
    axis = tune.axis
    local gains = { table.unpack(tune.best) }
    gains[axis] = math.max(0, gains[axis] + tune.direction * tune.dp[axis])
    prepare_trial(now, gains)
  end

  save_tune_results = function()
    if arming:is_armed() or not pending then
      return
    end
    local saved = 0
    for index, gain in pairs(pending.best) do
      local p_name, i_name = "CAN_P_" .. index, "CAN_I_" .. index
      local previous_p, previous_i = pget(p_name, 0.03), pget(i_name, 0.3)
      if previous_p == gain[1] and previous_i == gain[2] then
        saved = saved + 1
      elseif not param:set_and_save(p_name, gain[1]) then
        notice("P save failed ESC " .. index)
      elseif not param:set_and_save(i_name, gain[2]) then
        local restored = param:set_and_save(p_name, previous_p)
        notice(restored and ("I save failed ESC " .. index) or ("PI rollback failed ESC " .. index))
      else
        saved = saved + 1
      end
    end
    local path = pending.ground and "APM/CANTWDG.CSV" or "APM/CANTWD.CSV"
    local file = io.open(path, "wb")
    if file then
      local ok = file:write("esc,trial,p,i,mae_rpm,improved", pending.ground and ",global_mae_rpm\n" or "\n")
      for _, row in ipairs(pending.rows) do
        if not ok then break end
        ok = file:write(row)
      end
      if ok then ok = file:write("# ", pending.reason or "ended", "\n") end
      local closed = file:close()
      notice(ok and closed and (path .. " saved") or "log save failed")
    else
      notice("log open failed")
    end
    if pending.ground then
      local trace_path = "APM/CANTWDG.BIN"
      local trace = io.open(trace_path, "wb")
      local ok
      if trace then
        ok = trace:write("CTG1")
        for _, block in ipairs(pending.blocks) do
          if not ok then break end
          ok = trace:write(block)
        end
        local closed = trace:close()
        ok = ok and closed
      end
      notice(ok and (trace_path .. " saved") or "trace save failed")
      if pending.truncated then notice("trace capacity reached") end
    end
    local selector = pending.ground and 5 or 4
    pending = nil
    if pget("SCR_USER1", 0) == selector and not param:set_and_save("SCR_USER1", 0) then
      notice("reset SCR_USER1 manually")
    end
    notice(string.format("saved PI for %d ESCs", saved))
  end

  update_tune = function(armed, enabled, max_rpm, max_current, imax)
    local now = millis()
    local selector = pget("SCR_USER1", 0)
    local ground = selector == 5
    local selected = selector == 4 or ground
    local target = ground and (tune and tune.target or pget("CAN_TWD_G_RPM", 25)) or TARGET
    local eligible = enabled and vehicle:get_mode() == 0 and max_rpm > target
      and max_current > 0 and imax > 0 and pget("CAN_ENABLE", 0) == 1
      and pget("CAN_STEP_RESP", 0) ~= 1 and pget("CAN_STEP_RPM", 0) ~= 1
    if armed and not was_tune_armed and selected then
      inhibit_drive = true
      local selection = twiddle_selection()
      local max_trials = math.tointeger(pget("CAN_TWD_EPO", 12))
      if not max_trials or max_trials < 1 or max_trials > MAX_TRIALS then
        notice(string.format("CAN_TWD_EPO must be 1-%d", MAX_TRIALS))
      elseif ground and not (target >= 10 and target <= 40 and math.tointeger(target)) then
        notice("CAN_TWD_G_RPM must be 10-40")
      elseif eligible and ground then
        pending = { ground = true, best = {}, rows = {}, blocks = { "" }, bytes = 0 }
        tune = { ground = true, target = target, index = 1, last_esc = 1, start = now,
          max_current = max_current, max_rpm = max_rpm, imax = imax, max_trials = max_trials,
          best = {}, dp = {}, best_score = math.huge, trial = 0, axis = 1, direction = 1 }
        for index = 1, MAX_ESC do
          local p = math.max(0, pget("CAN_P_" .. index, 0.03))
          local i = math.max(0, pget("CAN_I_" .. index, 0.3))
          tune.best[2 * index - 1], tune.best[2 * index] = p, i
          tune.dp[2 * index - 1], tune.dp[2 * index] = math.max(0.001, p * 0.2), math.max(0.005, i * 0.2)
        end
        prepare_trial(now, tune.best)
      elseif eligible and selection then
        local first = selection <= MAX_ESC and selection or (selection == 5 and 1 or MAX_ESC)
        local last = selection <= MAX_ESC and selection or (selection == 5 and MAX_ESC or 1)
        pending = { best = {}, rows = {} }
        tune = { index = first, last_esc = last, increment = selection == 6 and -1 or 1,
          max_current = max_current, max_rpm = max_rpm, imax = imax, max_trials = max_trials }
        prepare_esc(now)
      else
        notice("check ESC, MANUAL, CAN and limits")
      end
    end
    was_tune_armed = armed
    if not tune then return end
    if not armed or not selected or not eligible or ground ~= (tune.ground == true) then
      stop(armed and "conditions changed" or "disarmed")
      return
    end
    if max_current ~= tune.max_current or max_rpm ~= tune.max_rpm or imax ~= tune.imax then
      stop("limits changed")
      return
    end
    if tune.ground then
      local steering, throttle = rc:get_pwm(1), rc:get_pwm(3)
      if not rc:has_valid_input() or not steering or not throttle
          or math.abs(steering - pget("RC1_TRIM", 1500)) > 100
          or math.abs(throttle - pget("RC3_TRIM", 1500)) > 100 then
        stop("RC input or pilot override")
        return
      end
    end
    local gear = math.max(pget("CAN_GEAR", 19.2032), 1)
    local all_stopped = true
    for index = 1, MAX_ESC do
      local fb = esc[index]
      if (tune.ground or (index >= math.min(tune.index, tune.last_esc) and index <= math.max(tune.index, tune.last_esc))
          or fb.detected) and not healthy(index - 1) then
        stop("feedback lost ESC " .. index)
        return
      end
      local signed_rpm = direction(index - 1) * fb.rotor_rpm / gear
      local wheel_speed = math.abs(signed_rpm)
      all_stopped = all_stopped and wheel_speed <= 1
      if wheel_speed >= max_rpm then
        stop(string.format("ESC %d RPM %.1f/%.1f", index, wheel_speed, max_rpm))
        return
      end
    end
    local rpm = direction(tune.index - 1) * esc[tune.index].rotor_rpm / gear
    local elapsed = as_number(now - tune.since)
    if tune.reference == nil then
      if (tune.ground and all_stopped) or (not tune.ground and math.abs(rpm) <= 1) then
        tune.rest_since = tune.rest_since or now
      else
        tune.rest_since = nil
      end
      if tune.rest_since and as_number(now - tune.rest_since) >= 1000 then
        set_reference(now, tune.ground and (tune.reverse_pending and -tune.target or tune.target) or -TARGET)
      elseif elapsed >= 15000 then
        stop("wheel did not stop")
      end
      return
    end
    local interval = as_number(now - tune.last)
    if interval >= FEEDBACK_TIMEOUT_MS then
      stop("control timing gap")
      return
    end
    local weight = math.max(0, math.min(elapsed, HOLD_MS) - math.max(elapsed - interval, 500))
    if tune.ground then
      for index, fb in ipairs(esc) do
        local measured = direction(index - 1) * fb.rotor_rpm / gear
        local error_ms = math.abs(tune.reference - measured) * weight
        tune.error_by_esc[index] = tune.error_by_esc[index] + error_ms
        tune.error_ms = tune.error_ms + error_ms / MAX_ESC
      end
    else
      tune.error_ms = tune.error_ms + math.abs(tune.reference - rpm) * weight
    end
    tune.weight_ms = tune.weight_ms + weight
    tune.last = now
    if elapsed >= HOLD_MS then
      if tune.ground and not tune.reverse_pending then
        tune.reverse_pending = true
        tune.reference, tune.since, tune.rest_since = nil, now, nil
        tune.trace_last = nil
        notice("ground waiting for stop")
      elseif not tune.ground and tune.reference < TARGET then
        set_reference(now, tune.reference + STEP)
      else
        complete_trial(now)
      end
    end
  end
  -- CTG1: ms, trial, reference (0.1 RPM), then RPM, commanded and measured mA per ESC.
  log_ground_sample = function()
    if not tune or not tune.ground or (not tune.reference and tune.trace_last) then return end
    local now = millis()
    if tune.trace_last and as_number(now - tune.trace_last) < 250 then return end
    tune.trace_last = now
    local values = {}
    for index, fb in ipairs(esc) do
      local offset = (index - 1) * 3
      values[offset + 1] = math.floor(fb.wheel_rpm * 10)
      values[offset + 2] = math.floor(fb.current_a * 1000)
      values[offset + 3] = math.floor(constrain(direction(index - 1) * fb.current_raw / RAW_PER_AMP * 1000,
        -32768, 32767))
    end
    local record = string.pack(TRACE_FORMAT, as_number(now - tune.start), tune.trial,
      (tune.reference or 0) * 10, table.unpack(values))
    if pending.bytes + #record > MAX_TRACE_BYTES then
      pending.truncated = true
      return
    end
    local blocks = pending.blocks
    if #blocks[#blocks] + #record > 1024 then
      blocks[#blocks + 1] = record
    else
      blocks[#blocks] = blocks[#blocks] .. record
    end
    pending.bytes = pending.bytes + #record
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
  elseif not was_armed and pget("SCR_USER1", 0) ~= 4 and pget("SCR_USER1", 0) ~= 5 and latched_can and vehicle:get_mode() == 0
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
        or (tune and (tune.ground or tune.index == i + 1)))
    if not active then
      fb.integrator = 0
      fb.last_error = 0
      fb.start_sign, fb.start_time, fb.start_motion = nil, nil, nil
      fb.target_rpm = 0
      fb.current_a = 0
    else
      local current_a
      if (step and not step.rpm) or (tune and (not tune.reference or tune.reference == 0)) then
        -- Identification and validation own the active ESC current.
        fb.integrator = 0
        fb.last_error = 0
        fb.start_sign, fb.start_time, fb.start_motion = nil, nil, nil
        fb.target_rpm = 0
        current_a = step and step.current or 0
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
        fb.target_rpm = tune and tune.reference or cmd * max_rpm
        local kp = tune and (tune.ground and tune.gains[2 * i + 1] or tune.P) or pget("CAN_P_" .. esc_number, 0.03)
        local ki = tune and (tune.ground and tune.gains[2 * i + 2] or tune.I) or pget("CAN_I_" .. esc_number, 0.3)
        local kd = pget("CAN_D_" .. esc_number, 0)
        local start_param = fb.target_rpm < 0 and "CAN_START_R_" or "CAN_START_F_"
        local breakaway = pget(start_param .. esc_number, 0)
        current_a = speed_current(fb, fb.target_rpm, kp, ki, kd, kff, dt, imax, max_current,
          breakaway, START_TIMEOUT_S)
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
    log_ground_sample()
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
  local twiddle = pget("SCR_USER1", 0) == 4 and pget("CAN_ENABLE", 0) == 1 and vehicle:get_mode() == 0
  local selection = twiddle and twiddle_selection()
  if twiddle and not selection then
    arming:set_aux_auth_failed(auth_id, "CAN_TWD_ESC must be 1-6")
    return
  end
  local single = selection and selection <= MAX_ESC
  local first, last = single and selection or 1, single and selection or MAX_ESC
  for i = first - 1, last - 1 do
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
  drain()
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
