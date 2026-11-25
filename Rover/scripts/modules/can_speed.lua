--[[
CAN wheel speed control: each wheel closes its own speed PI on its encoder
toward the rpm reference of its side, with the reference ramp (ESC_ACC),
prefilter (ESC_PREF), braking window (ESC_BRK), stall boost (ESC_STALL) and
side desaturation (ESC_OPT bit 2). Part of the CAN output (can_esc.lua).
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
Update the commanded side norms. Reactive path: each new mixer command applies
L/R immediately (no inter-sample interpolation).
--]]
local function update_command(now, dt)
  if not script_mixer_active() then
    local left, right = servo_side_norms()
    side[1].goal, side[2].goal = left, right
    side[1].interp_value, side[2].interp_value = left, right
    return
  end
  local seq = C.cmd_seq
  if C.cmd_seq_seen == nil then
    -- First value seen after boot or a switch is not a fresh command.
    C.cmd_seq_seen = seq
  elseif seq ~= C.cmd_seq_seen then
    C.cmd_seq_seen = seq
    C.cmd_seq_ms = now
    side[1].goal = constrain(C.cmd_l, -1, 1)
    side[2].goal = constrain(C.cmd_r, -1, 1)
    side[1].interp_value = side[1].goal
    side[2].interp_value = side[2].goal
  end
  if C.cmd_seq_ms == nil or now - C.cmd_seq_ms > K.CMD_STALE_MS then
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
  -- Stall boost: a wheel asked to move but still stopped (static friction,
  -- or the scrub of a pivot turn) gets its integrator raised at ESC_STALL A/s
  -- until it breaks away. The plain integrator needed about 1 s to reach the
  -- 10-12 A a pivot takes, and the yaw loop saw that as dead time.
  -- Stalled means the wheel has made under half its target and under 5 rpm,
  -- so a slow target (a yaw correction of a few degrees asks for 1-3 rpm)
  -- also breaks free, while a wheel that is accelerating normally is left alone.
  if k.stall > 0 and math.abs(target) >= K.STALL_MIN_RPM and
      math.abs(fb.wheel_rpm) < math.min(K.MOVING_RPM, math.max(K.STALL_NOISE_RPM, 0.5 * math.abs(target))) then
    local add = (target > 0 and 1 or -1) * k.stall * dt
    local before = fb.integrator
    fb.integrator = constrain(fb.integrator + add, -k.imax, k.imax)
    fb.boost = (fb.boost or 0) + (fb.integrator - before)
  elseif fb.boost ~= nil and fb.boost ~= 0 then
    -- Broke free: moving friction is below the breakaway current, so hand
    -- back part of the boost or the wheel jumps well past a slow target
    fb.integrator = constrain(fb.integrator - K.STALL_RELEASE * fb.boost, -k.imax, k.imax)
    fb.boost = 0
  end
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
  cached.stall = constrain(g("STALL"), 0, 200)
  -- Override the effective cache only; preserve the individual saved settings.
  -- Keep logging/zero-rpm hold bits of ESC_OPT and all hard current/safety limits.
  if g("RPM_AUX") < 0.5 then
    cached.acc = 0
    cached.pref = 0
    cached.brake = false
    cached.stall = 0
    cached.opt = cached.opt & (~4)
  end
  return cached
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
        if now - C.last_slip_warn >= 5000 then
          C.last_slip_warn = now
          gcs:send_text(MAV_SEVERITY.WARNING, string.format("ESC: slip on ESC %d (%.0f rpm from its side)", i + 1, diff))
        end
      end
    else
      slip_since[i + 1] = nil
    end
  end
  C.slip_active = any
end

function M.run_control()
  local now_us = micros()
  local dt = 0.01
  if C.last_us ~= nil then
    dt = constrain(as_number(now_us - C.last_us) * 1.0e-6, 0, 0.05)
  end
  C.last_us = now_us
  local now = now_ms()

  local k = read_gains(now)
  local hold_disarmed = (k.opt & 2) ~= 0
  local armed = arming:is_armed()
  local safety_live = not SRV_Channels:get_safety_state()
  local estop = SRV_Channels:get_emergency_stop()
  local outputs_ok = C.latched_can and safety_live and not estop and mask_valid() and all_healthy()
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

  -- nil without can_wheeltune.lua or while no tune owns the wheels
  local wheel_currents = M.wheel_tune and M.wheel_tune(dt, k, armed, enabled)
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
        -- Below 0.004 (0.5 rpm at ESC_RPM 120) the side is at rest; a slow yaw
        -- correction (1-3 rpm) must still be driven, not coasted
        local at_rest = math.abs(sd.interp_value) < K.REST_CMD and math.abs(fb.ref_rpm) < K.MOVING_RPM
            and math.abs(fb.wheel_rpm) < K.MOVING_RPM
        if at_rest then
          fb.integrator = 0
          fb.boost = 0
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
          ref = prefilter(fb, ref, dt, wheel_ti(i, k) * k.pref)
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
      C.sat_ms = now
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
        fb.boost = 0
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
  if not C.latched_can then
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

C.read_gains = read_gains
C.profile_wheel = profile_wheel
C.wheel_pi = wheel_pi
C.wheel_ti = wheel_ti
C.prefilter = prefilter

return true
