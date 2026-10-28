-- C620 / M3508 driver for stock ArduPilot Rover.
-- Copy this file to APM/scripts on the SD card. No custom firmware is required.
--
-- CAN_Px_DRIVER = 1 or 2
-- CAN_Px_BITRATE = 1000000
-- CAN_Dx_PROTOCOL = 10 (Scripting)
-- SCR_ENABLE = 1
-- SCR_USER5 = 0 so the other Lua mixer stays off and Rover drives the servos,
-- the same path the custom firmware used.
--
-- C620_CAN   0 = CAN off (PWM), 1 = CAN on. Change only while disarmed.
-- C620_MODE  0 = wheel speed PID, 1 = current follows the stick
-- C620_RPM   wheel rpm at full stick
-- C620_GEAR  rotor-to-wheel ratio (M3508 P19 = 19.2032)
-- C620_CUR   current limit, ampere
-- C620_P _I _D _FF _IMAX
-- C620_REV   bitmask, bit 0 = ESC1 ... bit 7 = ESC8. Use 9 on this robot.
-- C620_OPT   bit 0 = log DJIC, bit 1 = hold zero rpm while disarmed
--
-- Wheel distance is logged in DJIC.Pos. Stock firmware has no Lua hook into
-- WENC, so leave WRC_ENABLE = 0. The speed loop below is the closed loop.

local MAX_ESC = 8
local CMD_1_4 = 0x200
local CMD_5_8 = 0x1FF
local FB_BASE = 0x200
local COUNTS_PER_REV = 8192
local RAW_PER_AMP = 16384 / 20
local RAW_MAX = 16384
local FEEDBACK_TIMEOUT_MS = 100
local LOOP_MS = 5
local MAX_FRAMES = 25
-- was 72 with C620_OUT; bump so the Pixhawk creates C620_CAN on a fresh table
local PARAM_KEY = 73
local TELEM_CURRENT = 1 << 3
local TELEM_MOTOR_TEMP = 1 << 1

local MAV_SEVERITY = { CRITICAL = 2, WARNING = 4, INFO = 6 }

local driver = nil
local latched_can = true
local auth_id = nil
local last_switch_warn = uint32_t(0)
local last_lost_warn = uint32_t(0)
local last_driver_warn = uint32_t(0)
local last_log = uint32_t(0)
local last_telem = uint32_t(0)
local params_ready = false
local last_us = nil
local lost_mask = 0
local announced_driver = false

local script_norm = { 0, 0, 0, 0, 0, 0, 0, 0 }
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
  if not param:add_table(PARAM_KEY, "C620", 12) and param:get("C620_CAN") == nil then
    return false
  end
  local spec = {
    { 1, "_CAN", 1 },
    { 2, "_MODE", 0 },
    { 3, "_RPM", 400 },
    { 4, "_GEAR", 19.2032 },
    { 5, "_CUR", 10 },
    { 6, "_P", 0.03 },
    { 7, "_I", 0.3 },
    { 8, "_D", 0 },
    { 9, "_FF", 0 },
    { 10, "_IMAX", 10 },
    { 11, "_REV", 0 },
    { 12, "_OPT", 1 },
  }
  for n = 1, #spec do
    local item = spec[n]
    if not param:add_param(PARAM_KEY, item[1], item[2], item[3]) then
      gcs:send_text(MAV_SEVERITY.WARNING, string.format("DJICAN: failed to add C620%s", item[2]))
      return false
    end
  end
  params_ready = true
  gcs:send_text(MAV_SEVERITY.INFO, "DJICAN: C620_* parameters ready")
  return true
end

local function servo_function(idx)
  return math.floor(pget(string.format("SERVO%d_FUNCTION", idx + 1), 0))
end

local function is_throttle(idx)
  local fn = servo_function(idx)
  return fn == 70 or fn == 73 or fn == 74
end

local function direction(idx)
  local rev = math.floor(pget("C620_REV", 0))
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

-- Rover writes ThrottleLeft/Right as -1000..1000 and Throttle as -100..100.
local function servo_norm(idx)
  local fn = servo_function(idx)
  local scale
  if fn == 73 or fn == 74 then
    scale = 1000
  elseif fn == 70 then
    scale = 100
  else
    return 0
  end
  local scaled = SRV_Channels:get_output_scaled(fn) or 0
  local norm = scaled / scale
  if pget(string.format("SERVO%d_REVERSED", idx + 1), 0) > 0 then
    norm = -norm
  end
  return constrain(norm, -1, 1)
end

local function command_norm(idx)
  if script_fresh and as_number(millis() - script_ms) < 500 then
    return script_norm[idx + 1] or 0
  end
  return servo_norm(idx)
end

local function healthy(idx)
  local fb = esc[idx + 1]
  if not fb.detected or fb.last_ms == nil then
    return false
  end
  return as_number(millis() - fb.last_ms) < FEEDBACK_TIMEOUT_MS
end

local function apply_switch()
  local want_can = pget("C620_CAN", 1) >= 0.5
  if want_can == latched_can then
    return
  end
  if arming:is_armed() then
    local now = millis()
    if as_number(now - last_switch_warn) >= 5000 then
      last_switch_warn = now
      gcs:send_text(MAV_SEVERITY.WARNING, "DJICAN: disarm to switch C620_CAN")
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

local function run_control()
  local now_us = micros()
  local dt = 0.01
  if last_us ~= nil then
    dt = constrain(as_number(now_us - last_us) * 1.0e-6, 0, 0.05)
  end
  last_us = now_us

  local gear = math.max(pget("C620_GEAR", 19.2032), 1)
  local max_rpm = pget("C620_RPM", 400)
  local max_current = constrain(pget("C620_CUR", 10), 0, 20)
  local imax = constrain(pget("C620_IMAX", 10), 0, max_current)
  local kp = pget("C620_P", 0.03)
  local ki = pget("C620_I", 0.3)
  local kd = pget("C620_D", 0)
  local kff = pget("C620_FF", 0)
  local speed_mode = pget("C620_MODE", 0) < 0.5
  local hold_disarmed = (math.floor(pget("C620_OPT", 1)) & 2) ~= 0
  local armed = arming:is_armed()
  local safety_live = not SRV_Channels:get_safety_state()
  local estop = SRV_Channels:get_emergency_stop()
  local enabled = latched_can and safety_live and not estop and (armed or (speed_mode and hold_disarmed))

  local raw = { 0, 0, 0, 0, 0, 0, 0, 0 }
  local detected = 0
  for i = 0, MAX_ESC - 1 do
    local fb = esc[i + 1]
    local dir = direction(i)
    fb.wheel_rpm = dir * fb.rotor_rpm / gear
    if fb.detected then
      detected = detected | (1 << i)
    end
    local active = enabled and fb.have_angle and healthy(i)
    if not active then
      fb.integrator = 0
      fb.last_error = 0
      fb.target_rpm = 0
      fb.current_a = 0
    else
      local cmd = 0
      if armed then
        cmd = command_norm(i)
      end
      local current_a
      if speed_mode then
        fb.target_rpm = cmd * max_rpm
        local err = fb.target_rpm - fb.wheel_rpm
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
      else
        fb.target_rpm = 0
        fb.integrator = 0
        fb.last_error = 0
        current_a = constrain(cmd * max_current, -max_current, max_current)
      end
      fb.current_a = current_a
      raw[i + 1] = dir * current_a * RAW_PER_AMP
    end
  end

  -- PWM mode leaves the bus silent so the C620 can follow the servo pulse.
  if not latched_can then
    return
  end
  if (detected & 0x0F) ~= 0 then
    send_group(CMD_1_4, { raw[1], raw[2], raw[3], raw[4] })
  end
  if (detected & 0xF0) ~= 0 then
    send_group(CMD_5_8, { raw[5], raw[6], raw[7], raw[8] })
  end
end

local function hold_trim()
  -- MAIN OUT 1..4 are wired to the C620 PWM inputs. A pulse, even at trim,
  -- makes the ESC leave CAN. Zero width suppresses the pulse while CAN commands.
  for i = 0, MAX_ESC - 1 do
    local drive = is_throttle(i) or (script_fresh and i < 4)
    if drive then
      SRV_Channels:set_output_pwm_chan_timeout(i, 0, 300)
    end
  end
end

local function check_health()
  if not latched_can or not arming:is_armed() then
    lost_mask = 0
    return
  end
  local lost = 0
  for i = 0, MAX_ESC - 1 do
    if esc[i + 1].detected and is_throttle(i) and not healthy(i) then
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
    local need = is_throttle(i) or (script_fresh and i < 4)
    if need and not esc[i + 1].detected then
      arming:set_aux_auth_failed(auth_id, string.format("ESC %d not found", i + 1))
      return
    end
    if need and not healthy(i) then
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
  if (math.floor(pget("C620_OPT", 1)) & 1) == 0 then
    return
  end
  local now = millis()
  if as_number(now - last_log) < 20 then
    return
  end
  last_log = now
  local gear = math.max(pget("C620_GEAR", 19.2032), 1)
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

C620 = {}

function C620.output_is_can()
  return latched_can and driver ~= nil
end

function C620.set_norms(norms)
  for i = 1, MAX_ESC do
    script_norm[i] = norms[i] or 0
  end
  script_ms = millis()
  script_fresh = true
end

function C620.clear_script_norms()
  script_fresh = false
end

local function update()
  if not ensure_params() then
    return update, 1000
  end
  if driver == nil then
    driver = CAN:get_device(25)
    if driver == nil then
      local now = millis()
      if as_number(now - last_driver_warn) >= 5000 then
        last_driver_warn = now
        gcs:send_text(MAV_SEVERITY.WARNING, "DJICAN: set CAN_Dx_PROTOCOL to Scripting (10)")
      end
      return update, 1000
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
  write_log()
  return update, LOOP_MS
end

return update, 2000
