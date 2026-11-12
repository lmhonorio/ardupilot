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
-- Free parameter table key for the CAN_ settings.
local PARAM_KEY = 18
local TELEM_CURRENT = 1 << 3
local TELEM_MOTOR_TEMP = 1 << 1

local MAV_SEVERITY = { CRITICAL = 2, WARNING = 4, INFO = 6 }

local driver = nil
local latched_can = false
local auth_id = nil
local last_switch_warn = uint32_t(0)
local last_lost_warn = uint32_t(0)
local last_driver_warn = uint32_t(0)
local last_log = uint32_t(0)
local last_telem = uint32_t(0)
local params_ready = false
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
  if not param:add_table(PARAM_KEY, "CAN_", 21) and param:get("CAN_ENABLE") == nil then
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
    { 21, "STEP_RPM", 0 },
  }
  for i = 1, MAX_ESC do
    local base = 8 + (i - 1) * 3
    spec[#spec + 1] = { base + 1, "P_" .. i, 0.03 }
    spec[#spec + 1] = { base + 2, "I_" .. i, 0.3 }
    spec[#spec + 1] = { base + 3, "D_" .. i, 0 }
  end
  for n = 1, #spec do
    local item = spec[n]
    if not param:add_param(PARAM_KEY, item[1], item[2], item[3]) then
      gcs:send_text(MAV_SEVERITY.WARNING, string.format("DJICAN: failed to add CAN_%s", item[2]))
      return false
    end
  end
  params_ready = true
  gcs:send_text(MAV_SEVERITY.INFO, "DJICAN: CAN_* parameters ready")
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

-- Little-endian samples: CSR1 uses 9 bytes; CRP1 adds 2 bytes for target RPM.
local step
local pending_result
local was_armed = false
local inhibit_drive = false

local function save_step()
  local result = pending_result
  pending_result = nil
  local path = result.rpm and "APM/CANRPM.BIN" or "APM/CANSTEP.BIN"
  local file = io.open(path, "wb")
  local ok
  if file then
    local written = file:write(result.rpm and "CRP1" or "CSR1", table.concat(result.samples))
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

local function update_step(armed, enabled, max_rpm, max_current)
  local now = millis()
  local rpm_requested = pget("CAN_STEP_RPM", 0) == 1
  local current_requested = pget("CAN_STEP_RESP", 0) == 1
  if not armed then
    inhibit_drive = false
  elseif not was_armed and latched_can and vehicle:get_mode() == 0
      and (rpm_requested or current_requested) then
    inhibit_drive = true
    if rpm_requested and current_requested then
      gcs:send_text(MAV_SEVERITY.WARNING, "CAN step: select only one test and rearm")
      was_armed = true
      return
    end
    step = { index = 1, start = now, phase_start = now, samples = {},
      rpm = rpm_requested, param = rpm_requested and "CAN_STEP_RPM" or "CAN_STEP_RESP" }
    gcs:send_text(MAV_SEVERITY.INFO, "CAN step: starting ESC 1")
  end
  was_armed = armed
  if not step then
    return
  end
  local fb = esc[step.index]
  if not armed or not enabled or vehicle:get_mode() ~= 0 or pget(step.param, 0) ~= 1
      or not healthy(step.index - 1) or max_rpm < 15 or max_current <= 0
      or math.abs(fb.rotor_rpm / math.max(pget("CAN_GEAR", 19.2032), 1)) >= max_rpm then
    finish_step("aborted")
    return
  end

  local elapsed = as_number(now - step.phase_start)
  if not step.base then
    local target = step.rpm and math.min(20, max_rpm * 0.5) or math.min(40, max_rpm * 0.25)
    if math.abs(fb.wheel_rpm - target) > target * 0.1 then
      step.stable_since = nil
    else
      step.stable_since = step.stable_since or now
      if as_number(now - step.stable_since) >= 400 then
        local delta = math.min(0.5, max_current * 0.1)
        if not step.rpm and math.abs(fb.current_a) + delta >= max_current then
          finish_step("insufficient current headroom")
          return
        end
        step.base = fb.current_a
        step.delta = delta
        step.phase_start = now
      end
    end
    if not step.base and elapsed >= 3000 then
      finish_step("could not stabilize")
    end
  elseif elapsed >= (step.rpm and 2200 or 1000) then
    step.index = step.index + 1
    if step.index > MAX_ESC then
      if not param:set_and_save(step.param, 0) then
        gcs:send_text(MAV_SEVERITY.WARNING, "CAN step: could not reset " .. step.param)
      end
      finish_step("complete")
      return
    end
    step.phase_start = now
    step.base, step.delta, step.stable_since = nil, nil, nil
    step.last_sample = nil
  end
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
      and (not inhibit_drive or (step and step.index == i + 1))
    if not active then
      fb.integrator = 0
      fb.last_error = 0
      fb.target_rpm = 0
      fb.current_a = 0
    else
      local cmd = 0
      if step then
        local target = math.min(40, max_rpm * 0.25)
        if step.rpm then
          target = math.min(20, max_rpm * 0.5)
          local elapsed = as_number(millis() - step.phase_start)
          if step.base and elapsed >= 200 and elapsed < 1200 then
            target = target * 1.5
          end
        end
        cmd = target / max_rpm
      elseif armed then
        cmd = command_norm(i)
      end
      local current_a
      fb.target_rpm = cmd * max_rpm
      local err = fb.target_rpm - fb.wheel_rpm
      local esc_number = tostring(i + 1)
      local kp = pget("CAN_P_" .. esc_number, 0.03)
      local ki = pget("CAN_I_" .. esc_number, 0.3)
      local kd = pget("CAN_D_" .. esc_number, 0)
      local derivative = 0
      if dt > 0 then
        derivative = kd * (err - fb.last_error) / dt
      end
      fb.last_error = err
      local unsat = kff * fb.target_rpm + kp * err + fb.integrator + derivative
      local saturated = (unsat >= max_current and err > 0) or (unsat <= -max_current and err < 0)
      if not saturated then
        fb.integrator = constrain(fb.integrator + ki * err * dt, -imax, imax)
      end
      current_a = constrain(kff * fb.target_rpm + kp * err + fb.integrator + derivative, -max_current, max_current)
      if step and step.base then
        local now = millis()
        local elapsed = as_number(now - step.phase_start)
        if not step.rpm then
          current_a = constrain(step.base + (elapsed >= 200 and step.delta or 0), -max_current, max_current)
        end
        if not step.last_sample or as_number(now - step.last_sample) >= 10 then
          step.last_sample = now
          local sample = string.pack("<BI2i2i2i2", step.index,
            as_number(now - step.start), math.floor(fb.wheel_rpm * 10),
            math.floor(current_a * 1000), math.floor(constrain(dir * fb.current_raw / RAW_PER_AMP * 1000, -32768, 32767)))
          if step.rpm then
            sample = sample .. string.pack("<i2", math.floor(fb.target_rpm * 10 + 0.5))
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
  local saw = false
  for i = 0, MAX_ESC - 1 do
    if esc[i + 1].detected then
      saw = true
    end
    if not esc[i + 1].detected then
      arming:set_aux_auth_failed(auth_id, string.format("ESC %d not found", i + 1))
      return
    end
    if not healthy(i) then
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
      local dir = direction(i)
      esc_telem:update_rpm(i, dir * fb.rotor_rpm, 0)
      local data = ESCTelemetryData()
      data:motor_temp_cdeg(math.floor(fb.temperature * 100))
      data:current(math.abs(fb.current_raw) / RAW_PER_AMP)
      esc_telem:update_telem_data(i, data, TELEM_MOTOR_TEMP | TELEM_CURRENT)
    end
  end
end

local motor_can = {}

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
  return LOOP_MS
end

return motor_can
