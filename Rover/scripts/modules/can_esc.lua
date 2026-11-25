-- ESC CAN driver for DJI C620 / M3508 on stock ArduPilot Rover.
-- Module of rover.lua (APM/scripts/modules/can_esc.lua with can_state, can_speed,
-- can_wheeltune and can_report). No custom firmware is required.
-- Full behavior, parameters and state: Rover/docs/esc_can.md.
--
-- CAN_Px_DRIVER = 1 or 2, CAN_Px_BITRATE = 1000000, CAN_Dx_PROTOCOL = 10 (Scripting)
--
-- The mixer (mixer.lua) calls command(left, right) of this module and reads
-- selected() / pwm_blocked(). ESC_STAT still reports the driver state.
-- ESC_CMD_L, ESC_CMD_R and ESC_CMD_SEQ are no longer written; their slots stay.
--
-- ESC_CAN      0 = PWM (driver silent), 1 = CAN. Change only while disarmed.
-- ESC_MODE     0 = per-wheel speed PI, 1 = current follows the command
-- ESC_RPM_AUX  1 = configured RPM helpers; 0 = bypass ramp, prefilter, brake
--              window, stall boost and side desaturation. PID and hard limits stay active.
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
-- ESC_BRK      0 = current window is always ±ESC_CUR (opposing current allowed).
--              1 = while holding or speeding up above 5 wheel rpm, opposing
--              current is capped at ESC_BRKCUR % of ESC_CUR. Slowing, stopping
--              or reversing opens the full ±ESC_CUR window.
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
-- ESC_WT_BW    USER1=4/5 target bandwidth, rad/s (default 5; on the rover 8 overshot
--              15-188% on the first check, 6 gave 12-38%, 4.5 gave 1-7%). Higher = faster and
--              stiffer with bigger current peaks; capped by the measured lag.
-- ESC_WT_RPM   high plateau of USER1=4 and 5, wheel rpm. The low plateau is half.
--              Values below 60 stay on 120, so an old saved 30 does not take over.
-- ESC_GCS_HZ   live NAMED_VALUE_FLOAT stream to the GCS, Hz (0 = off, max 20):
--              ESCn_I measured current A, ESCn_ICMD commanded current A,
--              ESCn_RPM wheel rpm, ESC_TGT_L / ESC_TGT_R side target rpm,
--              ESCn_T motor temperature deg C (1 Hz).
-- ESC_STALL    stall boost, A/s (default 30, 0 = off): while a wheel has a speed
--              target of 5 rpm or more but is not turning, its integrator rises
--              at this rate (capped by ESC_IMAX) until it breaks away. Makes
--              pivots and starts answer in about 0.3 s instead of about 1 s.
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
local read_gains = nil

-- Shared state (creates ESC_*) and the speed loop. rover.lua loads the
-- optional can_wheeltune and can_report after this, one module per script run.
require("can_speed")
read_gains = C.read_gains

local D = {}

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
  if C.params_ready then
    return true
  end
  -- add_table returns true again after a script restart because the table already exists
  if not param:add_table(K.PARAM_KEY, K.PREFIX, #PARAM_SPEC) then
    local now = now_ms()
    if now - C.last_driver_warn >= 5000 then
      C.last_driver_warn = now
      gcs:send_text(MAV_SEVERITY.WARNING, "ESC: reboot to resize ESC_ parameter table")
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
  if not C.user_params_bound then
    C.user_params_bound = scr_user1:init("SCR_USER1") and scr_user5:init("SCR_USER5") and
      rc1_trim:init("RC1_TRIM") and rc3_trim:init("RC3_TRIM")
  end
  -- Commands from a previous boot must never look fresh.
  set_param("STAT", 0)
  C.params_ready = true
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

local function be_i16(hi, lo)
  local value = ((hi & 0xFF) << 8) | (lo & 0xFF)
  if value >= 32768 then
    value = value - 65536
  end
  return value
end

local function apply_switch()
  local want_can = g("CAN") >= 0.5
  if want_can == C.latched_can then
    return
  end
  if arming:is_armed() then
    local now = now_ms()
    if now - C.last_switch_warn >= 5000 then
      C.last_switch_warn = now
      gcs:send_text(MAV_SEVERITY.WARNING, "ESC: disarm to switch ESC_CAN")
    end
    return
  end
  C.latched_can = want_can
  M.gains_ms = 0
  M.prearm_ms = 0
  for i = 1, K.NUM_ESC do
    reset_esc(i)
  end
  reset_control()
  reset_command()
  tune_reset()
  C.last_us = nil
  C.lost_mask = 0
  C.sat_ms = nil
  if C.latched_can then
    -- Kill PWM first. The C620 must not see a servo pulse when CAN starts.
    kill_pwm()
    -- Drop stale RX from the PWM to CAN transition before accepting feedback.
    for _ = 1, K.MAX_FRAMES do
      if not C.driver:read_frame() then
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
    local frame = C.driver:read_frame()
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

function M.check_health()
  if not C.latched_can or not arming:is_armed() then
    C.lost_mask = 0
    return
  end
  local lost = 0
  for i = 0, K.MAX_FB - 1 do
    if esc[i + 1].detected and not healthy(i) then
      lost = lost | (1 << i)
    end
  end
  local now = now_ms()
  if lost ~= 0 and (lost ~= C.lost_mask or now - C.last_lost_warn >= 5000) then
    C.last_lost_warn = now
    for i = 0, K.MAX_FB - 1 do
      if (lost & (1 << i)) ~= 0 then
        local fb = esc[i + 1]
        gcs:send_text(MAV_SEVERITY.CRITICAL, string.format("ESC: ESC %d lost %.0fms (%.1fA %dC), motors stopped",
          i + 1, now - (fb.last_ms or now), math.abs(fb.current_raw) / K.RAW_PER_AMP, fb.temperature))
      end
    end
  end
  if lost == 0 and C.lost_mask ~= 0 then
    gcs:send_text(MAV_SEVERITY.INFO, "ESC: feedback restored")
  end
  C.lost_mask = lost
end

function M.prearm()
  if C.auth_id == nil then
    return
  end
  local now = now_ms()
  if now - M.prearm_ms < K.PREARM_MS then
    return
  end
  M.prearm_ms = now
  if not C.latched_can then
    arming:set_aux_auth_passed(C.auth_id)
    return
  end
  -- Name lookup walks the whole parameter tree. The old script does not appear mid-flight.
  if now - M.c620_ms >= 1000 then
    M.c620_ms = now
    M.c620_seen = param:get('C620_CAN') ~= nil
  end
  if M.c620_seen then
    arming:set_aux_auth_failed(C.auth_id, "delete c620_can.lua from APM/scripts")
    return
  end
  if not mask_valid() then
    arming:set_aux_auth_failed(C.auth_id, "ESC_LMASK needs ESCs on both sides")
    return
  end
  local saw = false
  for i = 0, K.MAX_FB - 1 do
    if esc[i + 1].detected then
      saw = true
    end
    if is_throttle(i) and not esc[i + 1].detected then
      arming:set_aux_auth_failed(C.auth_id, string.format("ESC %d not found (SERVO%d is a motor)", i + 1, i + 1))
      return
    end
    if is_throttle(i) and not healthy(i) then
      arming:set_aux_auth_failed(C.auth_id, string.format("ESC %d no feedback", i + 1))
      return
    end
  end
  if not saw then
    arming:set_aux_auth_failed(C.auth_id, "no ESC detected")
    return
  end
  arming:set_aux_auth_passed(C.auth_id)
end

function M.publish_stat()
  local stat = 0
  if C.latched_can and C.driver ~= nil then
    stat = stat | K.STAT_CAN
  end
  if C.sat_ms ~= nil and now_ms() - C.sat_ms < K.SAT_HOLD_MS then
    stat = stat | K.STAT_SAT
  end
  if C.tune.state == "ff" or C.tune.state == "step" or C.tune.state == "recover" or
    C.tune.state == "verify" or C.tune.state == "rest" then
    stat = stat | K.STAT_TUNE_RUN
  elseif C.tune.state == "done" then
    stat = stat | K.STAT_TUNE_OK
  elseif C.tune.state == "fail" then
    stat = stat | K.STAT_TUNE_FAIL
  end
  if C.slip_active then
    stat = stat | K.STAT_SLIP
  end
  if stat ~= C.published_stat then
    C.published_stat = stat
    set_param("STAT", stat)
  end
end

local function update()
  if not ensure_params() then
    return 1000
  end
  if C.auth_id == nil then
    C.auth_id = arming:get_aux_auth_id()
  end
  if C.driver == nil then
    C.driver = CAN:get_device(25)
    if C.driver == nil then
      local want_can = g("CAN") >= 0.5
      local now = now_ms()
      if want_can and now - C.last_driver_warn >= 5000 then
        C.last_driver_warn = now
        gcs:send_text(MAV_SEVERITY.WARNING, "ESC: ESC_CAN=1 but no scripting CAN driver (CAN_Dx_PROTOCOL=10)")
      end
      -- Refuse to arm instead of silently driving on PWM when CAN was asked for
      if C.auth_id ~= nil then
        if want_can then
          arming:set_aux_auth_failed(C.auth_id, "ESC_CAN=1 but CAN driver missing")
        else
          arming:set_aux_auth_passed(C.auth_id)
        end
      end
      M.publish_stat()
      return 1000
    end
    -- Keep only 0x200..0x20F (C620 feedback) in the script buffer so other
    -- traffic on the bus cannot push ESC frames out of it.
    C.driver:add_filter(0x7F0, 0x200)
    if not C.announced_driver then
      C.announced_driver = true
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
  if C.latched_can and not mask_valid() then
    local now = now_ms()
    if now - C.last_mask_warn >= 5000 then
      C.last_mask_warn = now
      gcs:send_text(MAV_SEVERITY.ERROR, "ESC: ESC_LMASK must put ESCs on both sides")
    end
  end
  -- Zero MAIN 1..4 every cycle. The override lasts 3 s, so a second call in
  -- the same cycle only cost VM time.
  if C.latched_can then
    kill_pwm()
  end
  drain()
  M.run_control()
  if C.latched_can then
    M.check_health()
  end
  if M.report_rx ~= nil then
    M.warn_selector_without_can()
    M.report_rx()
  end
  M.publish_stat()
  M.prearm()
  -- Spread EEPROM / telem / log so a single update stays under SCR_VM_I_COUNT
  M.drain_saves()
  if M.send_gcs ~= nil then
    local tick = now_ms() % 100
    if tick < 50 then
      M.send_telem()
    else
      M.write_log()
    end
    M.send_gcs()
  end
  return K.LOOP_MS
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
  if C.latched_can then
    kill_pwm()
    if C.driver ~= nil then
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
  if C.auth_id ~= nil then
    arming:set_aux_auth_failed(C.auth_id, "ESC script fault; reboot")
  end
end

-- One driver cycle for the scheduler in rover.lua; returns the delay to the next, ms
function D.step()
  if M.fault == nil then
    local ok, delay = pcall(update)
    if ok then
      return delay
    end
    M.fault = tostring(delay)
    M.fault_warn_ms = 0
  end
  pcall(M.safe_stop)
  local now = now_ms()
  if now - M.fault_warn_ms >= 5000 then
    M.fault_warn_ms = now
    gcs:send_text(MAV_SEVERITY.CRITICAL, "ESC: fault, motors stopped; reboot")
    gcs:send_text(MAV_SEVERITY.CRITICAL, "ESC: " .. string.sub(string.match(M.fault, "([%w_%-]+%.lua:%d+:.*)$") or M.fault, 1, 40))
  end
  return 100
end


--[[
Mixer interface.
selected()     the CAN driver drives the wheels (ESC_STAT bit 0)
pwm_blocked()  ESC_CAN = 1: no pulse may reach MAIN 1..4, even before the driver latched CAN
command(l, r)  side commands -1..1; stale after 500 ms (K.CMD_STALE_MS)
--]]
function D.selected()
  local stat = P.STAT
  if stat == nil then
    return false
  end
  return (math.floor(stat:get() or 0) & K.STAT_CAN) ~= 0
end

function D.pwm_blocked()
  local p = P.CAN
  if p ~= nil then
    return (p:get() or 0) >= 0.5
  end
  return (param:get(K.PREFIX .. "CAN") or 0) >= 0.5
end

function D.command(norm_l, norm_r)
  C.cmd_l = norm_l
  C.cmd_r = norm_r
  C.cmd_seq = (C.cmd_seq + 1) % 1000000
end

-- After a fault elsewhere in rover.lua: stop the wheels and refuse arming
D.safe_stop = M.safe_stop

-- The stand-alone driver started 2 s after load
D.FIRST_DELAY_MS = 2000

R.can = D
return D
