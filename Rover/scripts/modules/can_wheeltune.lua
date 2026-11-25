--[[
CAN wheel autotune (SCR_USER1 = 4 on the ground, 5 wheels in the air):
FF from two speed holds, then a current step forward and in reverse for each
wheel's Kv and lag, PI gains from them, and a closed-loop check.
Part of the CAN output (can_esc.lua). Remove this file to drop the wheel tune.
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
local read_gains, profile_wheel, wheel_pi, wheel_ti, prefilter = C.read_gains, C.profile_wheel, C.wheel_pi, C.wheel_ti, C.prefilter

-- Everything this module adds to the shared M table, so unload() can take it
-- back out and the memory is freed (rover.lua loads it only for SCR_USER1 = 4/5)
local M_before = {}
for key in pairs(M) do
  M_before[key] = true
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
  if C.tune.mode ~= nil and C.tune.mode ~= sel then
    return false
  end
  return armed and vehicle:get_mode() == K.HOLD_MODE
end

local function tune_fail(msg)
  C.tune.state = "fail"
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
    esc[i].boost = 0
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
  C.tune.t = 0
  C.tune.count = 0
  C.tune.sum_current = { 0, 0, 0, 0 }
  C.tune.sum_rpm = { 0, 0, 0, 0 }
  if zero_ref then
    M.clear_wheel_pi()
  else
    for i = 1, K.MAX_FB do
      esc[i].integrator = 0
      esc[i].boost = 0
      esc[i].last_error = 0
    end
  end
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
K.STEP_MAX_DEV = 0.5     -- a wheel stops its step once it moved this fraction of the plateau
K.STEP_MIN_N = 5         -- samples needed for a slope
K.STEP_RETRIES = 2       -- step again at half current when a wheel had too few samples
K.LAG_MIN = 0.02         -- lag floor for the bandwidth cap
K.LAG_DEFAULT = 0.03     -- lag when no wheel gave a usable intercept (CAN + gearbox, measured 34 ms)
K.STEP_NEG_LAG = 0.05    -- a fitted lag down to -50 ms is read as 0 (noise)
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
  return esc[w].detected and (C.tune.only == nil or C.tune.only == w)
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
  local ok, ps, is_, fs = M.save_wheel_gains(C.tune.rows)
  if not ok then
    tune_fail(ps)
    return
  end
  M.queue_save(scr_user1, 0)
  M.clear_wheel_pi()
  C.tune.state = "done"
  M.announce_saved(ps, is_, fs)
end

function M.tune_log_wheels(target, cur)
  if logger == nil then
    return
  end
  local ph = (C.tune.state == "step" and 2) or (C.tune.state == "verify" and 3) or 1
  for i = 1, K.NUM_ESC do
    if M.on(i) then
      logger:write("DJWT", "Side,Dir,Ph,Tgt,Rpm,Cur,Cyc", "BbBfffB",
        i, C.tune.dir or 0, ph, target, esc[i].wheel_rpm, cur[i], C.tune.tries or 0)
    end
  end
end

-- One current step: sign +1 speeds the wheels up (in the tune direction), -1 slows them
function M.step_begin(sign)
  local f = C.tune.fit
  C.tune.state = "step"
  C.tune.step_sign = sign
  C.tune.t = 0
  for w = 1, K.NUM_ESC do
    f.n[w], f.st[w], f.sr[w], f.stt[w], f.str[w] = 0, 0, 0, 0, 0
    f.r0[w] = C.tune.dir * esc[w].wheel_rpm
    f.done[w] = false
  end
end

-- Close one step: least-squares line rpm = a t + b on each wheel's moving samples
function M.step_close()
  local f = C.tune.fit
  local s = C.tune.step_sign
  for w = 1, K.NUM_ESC do
    local n = f.n[w]
    f.dn[w] = n
    if M.on(w) and n >= K.STEP_MIN_N then
      local den = n * f.stt[w] - f.st[w] * f.st[w]
      if den > 0 then
        local a = (n * f.str[w] - f.st[w] * f.sr[w]) / den
        local b = (f.sr[w] - a * f.st[w]) / n
        local kv = s * a / C.tune.step_i
        local lag = (a ~= 0) and (f.r0[w] - b) / a or -1
        f.dn[w], f.dkv[w], f.dlag[w] = n, kv, lag
        -- A light wheel answers almost at once; with rpm noise the line can
        -- cross the start rpm a little before the step. Read that as no lag.
        if finite(lag) and lag < 0 and lag > -K.STEP_NEG_LAG then
          lag = 0
        end
        -- Kv (the slope) and the lag (the intercept) are kept apart: a wheel
        -- already drifting when the step starts gives a good slope but a
        -- meaningless intercept (seen: n 18 kv 29 lag -0.091, then failed)
        if finite(kv) and kv > 0 then
          f.kv[w] = f.kv[w] + kv
          f.kvn[w] = f.kvn[w] + 1
          if finite(lag) and lag >= 0 and lag <= 0.3 then
            f.lag[w] = f.lag[w] + lag
            f.lagn[w] = f.lagn[w] + 1
          end
        end
      end
    end
  end
end

-- Gains for every wheel from its Kv, the median lag and the current bandwidth scale
function M.step_gains(k)
  local f = C.tune.fit
  local lags = M.fixed_list()
  local nl = 0
  for w = 1, K.NUM_ESC do
    if M.on(w) then
      if f.kvn[w] == 0 then
        return false, string.format("ESC %d no step response (n %d kv %.0f lag %.3f)",
          w, f.dn[w], f.dkv[w], f.dlag[w]), w
      end
      if f.lagn[w] > 0 then
        nl = nl + 1
        lags[nl] = f.lag[w] / f.lagn[w]
      end
    end
  end
  -- Median of the wheels with a usable intercept, never below 20 ms: a lag
  -- read as 0 (seen: T 0.001) lifted the bandwidth cap
  local lag = M.median(M.head(lags, nl)) or K.LAG_DEFAULT
  lag = math.max(lag, K.LAG_MIN)
  local bw = constrain(g("WT_BW"), 1, 50)
  if lag > 0 then
    bw = math.min(bw, 1 / (K.WC_LAG * lag))
  end
  C.tune.lag = lag
  C.tune.wc = bw * C.tune.bw_scale
  local ti = K.TI_WC / C.tune.wc
  for w = 1, K.NUM_ESC do
    if M.on(w) then
      local kv = f.kv[w] / f.kvn[w]
      local kp = C.tune.wc / kv
      local ki = kp / ti
      if not M.gains_sane(kp, ki, k) then
        return false, string.format("ESC %d gains out of range (Kv %.0f)", w, kv)
      end
      C.tune.tkp[w], C.tune.tki[w] = kp, ki
    end
  end
  return true
end

-- Start the check on the low plateau with each integrator already holding the
-- current measured there (minus its FF). From zero, a low-gain wheel stalls on
-- static friction, the integrator winds up and the wheel overshoots on breakaway.
function M.verify_begin(k)
  C.tune.state = "verify"
  C.tune.vphase = "hold"
  C.tune.t = 0
  for w = 1, K.NUM_ESC do
    C.tune.peak[w] = 0
    local fb = esc[w]
    fb.last_error = 0
    fb.integrator = 0
    fb.boost = 0
    if M.on(w) and C.tune.lo_i ~= nil then
      fb.integrator = constrain(C.tune.lo_i[w] - C.tune.tff[w] * C.tune.lo_rpm[w], -k.imax, k.imax)
    end
  end
  C.tune.imax = 0
end

function M.tune_start_direction(dir)
  C.tune.dir = dir
  C.tune.state = "ff"
  C.tune.ff_level = 1
  C.tune.lo_i = nil
  C.tune.lo_rpm = nil
  C.tune.base = { 0, 0, 0, 0 }
  C.tune.rpm = { 0, 0, 0, 0 }
  C.tune.bw_scale = 1
  C.tune.step_tries = 0
  C.tune.tries = 0
  -- Constructors: every list at its final size, no growth during the tune
  C.tune.fit = { n = { 0, 0, 0, 0 }, st = { 0, 0, 0, 0 }, sr = { 0, 0, 0, 0 }, stt = { 0, 0, 0, 0 },
    str = { 0, 0, 0, 0 }, r0 = { 0, 0, 0, 0 }, kv = { 0, 0, 0, 0 }, kvn = { 0, 0, 0, 0 },
    lag = { 0, 0, 0, 0 }, lagn = { 0, 0, 0, 0 }, dn = { 0, 0, 0, 0 }, dkv = { 0, 0, 0, 0 }, dlag = { 0, 0, 0, 0 },
    done = { false, false, false, false } }
  C.tune.tkp = { 0, 0, 0, 0 }
  C.tune.tki = { 0, 0, 0, 0 }
  C.tune.tff = { 0, 0, 0, 0 }
  C.tune.peak = { 0, 0, 0, 0 }
  -- Best check so far: shrinking the bandwidth does not always help (static
  -- friction with low gains), so the lowest overshoot is what gets saved
  C.tune.best_os = math.huge
  C.tune.bkp = { 0, 0, 0, 0 }
  C.tune.bki = { 0, 0, 0, 0 }
  C.tune.best_wc = 0
  C.tune.kt = nil
  M.tune_reset_window(true)
end

-- Store this direction's gains and move on: reverse, the next wheel (USER1=5) or save
function M.direction_done()
  local row = C.tune.rows[C.tune.dir > 0 and 1 or 2]
  for w = 1, K.NUM_ESC do
    if M.on(w) then
      row[w] = { P = C.tune.tkp[w], I = C.tune.tki[w], ff = C.tune.tff[w] }
    end
  end
  if C.tune.dir == C.tune.first then
    M.tune_start_direction(-C.tune.first)
  elseif C.tune.only ~= nil and M.next_wheel(C.tune.only) ~= nil then
    C.tune.only = M.next_wheel(C.tune.only)
    tune_message(MAV_SEVERITY.INFO, string.format("next ESC %d", C.tune.only))
    M.tune_start_direction(C.tune.first)
    -- Let the previous wheel spin down before the next one starts
    C.tune.state = "rest"
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
    if C.tune.state ~= "idle" and C.tune.state ~= "done" and C.tune.state ~= "fail" then
      tune_message(MAV_SEVERITY.WARNING, "aborted (left HOLD, disarmed or selector changed)")
    end
    if C.tune.state ~= "idle" then
      tune_reset()
      M.clear_wheel_pi()
    end
    return nil
  end
  if C.tune.state == "done" or C.tune.state == "fail" then
    return nil
  end
  if not k.speed_mode then
    if C.tune.state == "idle" then
      tune_fail("ESC_MODE must be 0")
    end
    return nil
  end
  if C.tune.state == "idle" then
    if not enabled then
      return nil
    end
    if not rc_centered() then
      local now = now_ms()
      if now - C.last_tune_warn >= 5000 then
        C.last_tune_warn = now
        tune_message(MAV_SEVERITY.WARNING, "center the sticks to start")
      end
      return nil
    end
    C.tune.mode = selector()
    C.tune.rows = { {}, {} }
    C.tune.only = nil
    -- Start with the rover's physical forward. The mixer's THRO_INV = 1 says
    -- forward is negative wheel rpm on this rover; positive rpm (the old
    -- "fwd") drove it backward first. Both directions are still tuned.
    C.tune.first = ((param:get("THRO_INV") or 0) >= 0.5) and -1 or 1
    if C.tune.mode == K.BRK_SELECTOR then
      C.tune.only = M.next_wheel(0)
      if C.tune.only == nil then
        tune_fail("no ESC detected")
        return nil
      end
    end
    M.tune_start_direction(C.tune.first)
    local lo, hi = M.tune_levels(k)
    tune_message(MAV_SEVERITY.NOTICE, string.format("%s step tune %.0f/%.0f rpm, step %.1fA, BW %.1f",
      C.tune.only and "in the air, one wheel," or "ground", lo, hi, M.relay_amp(k), constrain(g("WT_BW"), 1, 50)))
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
  local dir = C.tune.dir
  local dir_name = (dir == C.tune.first) and "fwd" or "rev"
  local cur = M.zero_currents()
  C.tune.t = C.tune.t + dt
  local target = dir * (C.tune.ff_level == 1 and lo_rpm or hi_rpm)

  if C.tune.state == "rest" then
    if C.tune.t >= K.BRK_REST_S then
      M.tune_start_direction(dir)
    end
    return cur
  elseif C.tune.state == "ff" then
    local any = false
    for i = 0, K.NUM_ESC - 1 do
      local fb = esc[i + 1]
      if M.on(i + 1) then
        any = true
        profile_wheel(i, target, dt)
        cur[i + 1] = wheel_pi(i, fb.ref_rpm, dt, k)
        if C.tune.t >= K.TUNE_SETTLE_S then
          C.tune.sum_current[i + 1] = C.tune.sum_current[i + 1] + cur[i + 1]
          C.tune.sum_rpm[i + 1] = C.tune.sum_rpm[i + 1] + fb.wheel_rpm
        end
      end
    end
    if not any then
      tune_fail("no ESC detected")
      return M.zero_currents()
    end
    if C.tune.t >= K.TUNE_SETTLE_S then
      C.tune.count = C.tune.count + 1
    end
    if C.tune.t >= K.TUNE_SETTLE_S + K.TUNE_AVERAGE_S then
      local means_i, means_r = { 0, 0, 0, 0 }, { 0, 0, 0, 0 }
      for i = 0, K.NUM_ESC - 1 do
        if M.on(i + 1) then
          local mean_rpm = C.tune.sum_rpm[i + 1] / C.tune.count
          if math.abs(mean_rpm - target) > 0.3 * math.abs(target) then
            tune_fail(string.format("ESC %d did not reach %.0f rpm (%.0f)", i + 1, target, mean_rpm))
            return M.zero_currents()
          end
          means_i[i + 1] = C.tune.sum_current[i + 1] / C.tune.count
          means_r[i + 1] = mean_rpm
          esc[i + 1].ref_rpm = target
        end
      end
      if C.tune.ff_level == 1 then
        C.tune.lo_i = means_i
        C.tune.lo_rpm = means_r
        C.tune.ff_level = 2
        M.tune_reset_window(false)
      else
        C.tune.base = means_i
        C.tune.rpm = means_r
        for w = 1, K.NUM_ESC do
          if M.on(w) then
            local ff = M.ff_slope(C.tune.lo_i[w], C.tune.lo_rpm[w], means_i[w], means_r[w])
            if ff == nil or not finite(ff) or ff < 0 then
              tune_fail(string.format("ESC %d rpm too low for FF", w))
              return M.zero_currents()
            end
            C.tune.tff[w] = ff
          end
        end
        C.tune.step_i = M.relay_amp(k)
        M.step_begin(1)
      end
    end
  elseif C.tune.state == "step" then
    local s = C.tune.step_sign
    local f = C.tune.fit
    local all_done = true
    for w = 1, K.NUM_ESC do
      local fb = esc[w]
      if M.on(w) then
        local r = dir * fb.wheel_rpm
        -- Each wheel has its own guard, from the rpm this step started at: a
        -- light wheel that gets there first goes back to its hold current
        -- while the heavier ones keep their step (one shared guard ended the
        -- step for every wheel before the heavy one had moved)
        if not f.done[w] and math.abs(r - f.r0[w]) > K.STEP_MAX_DEV * hi_rpm then
          f.done[w] = true
        end
        if f.done[w] then
          cur[w] = constrain(C.tune.base[w], -k.max_current, k.max_current)
        else
          all_done = false
          cur[w] = constrain(C.tune.base[w] + dir * s * C.tune.step_i, -k.max_current, k.max_current)
          -- Only samples already moving the way the step pushes: right after a
          -- reversal the wheel still runs on for the lag time
          if s * (r - f.r0[w]) >= K.STEP_MOVE_RPM then
            local t = C.tune.t
            f.n[w] = f.n[w] + 1
            f.st[w] = f.st[w] + t
            f.sr[w] = f.sr[w] + r
            f.stt[w] = f.stt[w] + t * t
            f.str[w] = f.str[w] + t * r
          end
        end
      end
    end
    if all_done or C.tune.t >= K.STEP_S then
      M.step_close()
      if s > 0 then
        M.step_begin(-1)
      else
        -- Recover on the plateau: reference already at it and each integrator
        -- holding the plateau current. Clearing them restarted the reference
        -- ramp from 0 rpm and braked the rover hard.
        C.tune.state = "recover"
        C.tune.t = 0
        for w = 1, K.NUM_ESC do
          local fb = esc[w]
          fb.last_error = 0
          fb.ref_rpm = target
          fb.ref_f = target
          if M.on(w) then
            local kff = k.kff_w[w] > 0 and k.kff_w[w] or k.kff
            fb.integrator = constrain(C.tune.base[w] - kff * target, -k.imax, k.imax)
          end
        end
      end
    end
  elseif C.tune.state == "recover" then
    for i = 0, K.NUM_ESC - 1 do
      if M.on(i + 1) then
        profile_wheel(i, target, dt)
        cur[i + 1] = wheel_pi(i, esc[i + 1].ref_rpm, dt, k)
      end
    end
    if C.tune.t >= K.RECOVER_S then
      local ok, why, short = M.step_gains(k)
      if not ok and short ~= nil and C.tune.step_tries < K.STEP_RETRIES then
        -- A light, fast wheel reaches its rpm guard in a few loop cycles and
        -- the fit gets too few points. Step again with half the current;
        -- wheels already measured just add to their average.
        C.tune.step_tries = C.tune.step_tries + 1
        C.tune.step_i = C.tune.step_i * 0.5
        tune_message(MAV_SEVERITY.INFO, string.format("%s ESC %d too fast for the step; again at %.1fA",
          dir_name, short, C.tune.step_i))
        M.step_begin(1)
        return cur
      end
      if not ok then
        tune_fail(why)
        return M.zero_currents()
      end
      local f = C.tune.fit
      local kvs = { 0, 0, 0, 0 }
      for w = 1, K.NUM_ESC do
        kvs[w] = f.kvn[w] > 0 and f.kv[w] / f.kvn[w] or 0
      end
      tune_message(MAV_SEVERITY.INFO, string.format("%s Kv %.0f %.0f %.0f %.0f T %.3f",
        dir_name, kvs[1], kvs[2], kvs[3], kvs[4], C.tune.lag))
      M.verify_begin(k)
    end
  elseif C.tune.state == "verify" then
    -- New gains through a proxy of the cached gains, with ESC_ACC and ESC_PREF as when driving
    local kt = C.tune.kt
    if kt == nil then
      kt = setmetatable({ kp_w = C.tune.tkp, ki_w = C.tune.tki, kff_w = C.tune.tff }, { __index = k })
      C.tune.kt = kt
    end
    local ref_goal = dir * (C.tune.vphase == "hold" and lo_rpm or hi_rpm)
    for i = 0, K.NUM_ESC - 1 do
      local fb = esc[i + 1]
      if M.on(i + 1) then
        profile_wheel(i, ref_goal, dt)
        local ref = fb.ref_rpm
        -- tf=0 passes through and tracks filter state while bypassed, so
        -- re-enabling the helpers cannot resume an old reference.
        ref = prefilter(fb, ref, dt, (C.tune.tkp[i + 1] / C.tune.tki[i + 1]) * k.pref)
        cur[i + 1] = wheel_pi(i, ref, dt, kt)
        if C.tune.vphase == "meas" then
          local r = dir * fb.wheel_rpm
          if r > C.tune.peak[i + 1] then
            C.tune.peak[i + 1] = r
          end
          if math.abs(cur[i + 1]) > C.tune.imax then
            C.tune.imax = math.abs(cur[i + 1])
          end
        end
      end
    end
    if C.tune.vphase == "hold" and C.tune.t >= K.VERIFY_HOLD_S then
      C.tune.vphase = "meas"
      C.tune.t = 0
    elseif C.tune.vphase == "meas" and C.tune.t >= K.VERIFY_MEAS_S then
      local os = 0
      for w = 1, K.NUM_ESC do
        if M.on(w) then
          os = math.max(os, (C.tune.peak[w] - hi_rpm) / (hi_rpm - lo_rpm))
        end
      end
      C.tune.tries = C.tune.tries + 1
      tune_message(MAV_SEVERITY.INFO, string.format("%s check %d: BW %.1f overshoot %.0f%% peak %.1fA",
        dir_name, C.tune.tries, C.tune.wc, math.max(os, 0) * 100, C.tune.imax))
      local improved = os < C.tune.best_os
      if improved then
        C.tune.best_os, C.tune.best_wc = os, C.tune.wc
        for w = 1, K.NUM_ESC do
          C.tune.bkp[w], C.tune.bki[w] = C.tune.tkp[w], C.tune.tki[w]
        end
      end
      if os > K.VERIFY_OS and C.tune.tries < K.VERIFY_TRIES and improved then
        C.tune.bw_scale = C.tune.bw_scale * K.BW_SHRINK
        local ok, why = M.step_gains(k)
        if not ok then
          tune_fail(why)
          return M.zero_currents()
        end
        M.verify_begin(k)
      else
        for w = 1, K.NUM_ESC do
          C.tune.tkp[w], C.tune.tki[w] = C.tune.bkp[w], C.tune.bki[w]
        end
        if C.tune.best_os > K.VERIFY_OS then
          tune_message(MAV_SEVERITY.WARNING, string.format("%s best overshoot %.0f%%; keeping BW %.1f",
            dir_name, C.tune.best_os * 100, C.tune.best_wc))
        end
        tune_message(MAV_SEVERITY.INFO, M.fmt4(dir_name .. " P", C.tune.tkp))
        tune_message(MAV_SEVERITY.INFO, M.fmt4(dir_name .. " I", C.tune.tki))
        M.direction_done()
        return M.zero_currents()
      end
    end
  end
  M.tune_log_wheels(target, cur)
  return cur
end

local M_added = {}
for key in pairs(M) do
  if not M_before[key] then
    M_added[#M_added + 1] = key
  end
end
M_before = nil

--[[
Unload: only while no tune runs (C.tune.state "idle"). The speed loop then
runs without M.wheel_tune, as when this file is not installed.
--]]
local WT = {}
function WT.idle()
  return C.tune.state == "idle"
end
function WT.unload()
  for i = 1, #M_added do
    M[M_added[i]] = nil
  end
end

return WT
