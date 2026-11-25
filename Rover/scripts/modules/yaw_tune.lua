--[[
Yaw PI autotune (SCR_USER1 = 2): relay test in HOLD around the current
heading. Saves SCR_USER2 (P) and SCR_USER3 (I), then runs the yaw_align
verification sequence. On PWM it also saves the turn breakaway as THRO_MIN_LUA.
--]]
if ... == nil then
  return
end

local R = require("rover_common")
local Autotune = require("autotune")
local funcs = require("functions")
local YA = require("yaw_align")
local MAV_SEVERITY = R.MAV_SEVERITY
local DRIVING_MODES = R.DRIVING_MODES
local TUNE_MODE = R.TUNE_MODE

local YT = {}

local TUNE_SELECT_PARAM = 'SCR_USER1'
local AUTOTUNE_PERIOD_MS = YA.UPDATE_PERIOD_MS
local YAW_RATE_THRESH_RAD_S = YA.YAW_RATE_THRESH_RAD_S

-- Relay output ramp and run limits
local S = {
  hold_hint_ms = -5000,
  still_s = 0,
  max_relay = 1,
  ramp_rate = 0,
  RAMP_STILL_S = 1.0,   -- time below the ramp turn rate before raising the output
  RAMP_STEP = 0.05,     -- raise by 5 % of the output range per step
  TUNE_TIMEOUT_S = 60,  -- relay measuring time, as in modules/autotune.lua
  MOVE_RATE = math.rad(5), -- turn rate that counts as the rover really turning
  still_lvl = nil,      -- last ramp output that left the rover still
  move_lvl = nil,       -- first ramp output that turned it
  last_tick_ms = 0,
  last_s_out = 0,
  last_progress = 0,
  last_log_s = 0,
  target_rad = nil,
  done = false,
}
YT.S = S

-- Parameters in the YAWT_ table created by yaw_align.lua
local KEY = YA.PARAM_TABLE_KEY
assert(param:add_param(KEY, 1, 'MAX_ANG_ERR', 90), 'could not add yaw autotune error limit')
--[[
  // @Param: YAWT_MAX_ANG_ERR
  // @DisplayName: Yaw autotune abort angle
  // @Description: Abort yaw autotune if the yaw error exceeds this angle
  // @Units: deg
  // @Range: 5 90
  // @User: Standard
--]]
local yaw_max_error_param = Parameter()
assert(yaw_max_error_param:init('YAWT_MAX_ANG_ERR'), 'could not bind YAWT_MAX_ANG_ERR parameter')

--[[
  // @Param: YAWT_MAX_OUT
  // @DisplayName: Yaw autotune maximum motor output
  // @Description: Highest steering command the yaw relay may use, as a percentage of the THRO_MAX_LUA output range. The relay starts at YAWT_MIN_OUT.
  // @Units: %
  // @Range: 1 100
  // @User: Standard
--]]
assert(param:add_param(KEY, 2, 'MAX_OUT', 100), 'could not add yaw autotune max output parameter')
local yaw_max_out_param = Parameter()
assert(yaw_max_out_param:init('YAWT_MAX_OUT'), 'could not bind YAWT_MAX_OUT parameter')

--[[
  // @Param: YAWT_MAX_ANG
  // @DisplayName: Yaw autotune relay switching angle
  // @Description: Heading error on either side of the starting heading where the relay reverses. Must be below YAWT_MAX_ANG_ERR.
  // @Units: deg
  // @Range: 1 89
  // @User: Standard
--]]
assert(param:add_param(KEY, 4, 'MAX_ANG', 45), 'could not add yaw autotune switching angle parameter')
local yaw_max_angle_param = Parameter()
assert(yaw_max_angle_param:init('YAWT_MAX_ANG'), 'could not bind YAWT_MAX_ANG parameter')

--[[
  // @Param: YAWT_MIN_OUT
  // @DisplayName: Yaw autotune minimum motor output
  // @Description: Relay output the yaw autotune starts with, as a percentage of the THRO_MAX_LUA output range. Before the first relay cycle, while the rover turns slower than needed to finish YAWT_MAX_CYCLE cycles of 4 x YAWT_MAX_ANG in about 36 s (checked over 1 s), the output rises in 5% steps up to YAWT_MAX_OUT, then stays fixed for the measurement. Must be at most YAWT_MAX_OUT.
  // @Units: %
  // @Range: 1 100
  // @User: Standard
--]]
assert(param:add_param(KEY, 5, 'MIN_OUT', 5), 'could not add yaw autotune min output parameter')
local yaw_min_out_param = Parameter()
assert(yaw_min_out_param:init('YAWT_MIN_OUT'), 'could not bind YAWT_MIN_OUT parameter')

--[[
  // @Param: YAWT_MAX_CYCLE
  // @DisplayName: Yaw autotune total relay cycles
  // @Description: Total full cycles to complete, including the first two discarded cycles
  // @Range: 3 60
  // @Increment: 1
  // @User: Standard
--]]
assert(param:add_param(KEY, 6, 'MAX_CYCLE', 6), 'could not add yaw autotune cycle parameter')
local yaw_max_cycle_param = Parameter()
assert(yaw_max_cycle_param:init('YAWT_MAX_CYCLE'), 'could not bind YAWT_MAX_CYCLE parameter')

local autotune = Autotune:new({ relay = 0.20 })
YT.autotune = autotune

local function apply(t, s, timeout_ms)
  return R.mixer.apply(t, s, timeout_ms)
end

--[[
Save the measured turn breakaway as THRO_MIN_LUA. The yaw PI output for a
small heading error is far below the output that starts the rover turning
(about 35-40% on the ground), so without this the last degrees are left to
the slow integrator. THRO_MIN_LUA is the kick used when the rover is stalled
off target.
--]]
local function saveBreakaway()
  if R.can_selected() then
    -- On CAN the wheel speed loop breaks the rover free (ESC_STALL); the PWM
    -- kick would only add a speed jump
    gcs:send_text(MAV_SEVERITY.INFO, "YawTune: CAN path, breakaway left to ESC_STALL; THRO_MIN_LUA kept")
    return
  end
  local still, move = S.still_lvl, S.move_lvl
  if move == nil or still == nil then
    gcs:send_text(MAV_SEVERITY.INFO, "YawTune: breakaway not seen (turned at the first output); THRO_MIN_LUA kept")
    return
  end
  if R.pwm == nil then
    return
  end
  R.pwm.save_breakaway(still, move)
end

--[[
Yaw PI autotune in HOLD mode.
-- @return bool - true while the autotune owns the outputs
--]]
local function run()
  if S.done then return false end
  local tick_due
  tick_due, S.last_tick_ms = YA.tick_due(S.last_tick_ms)
  if not tick_due then
    apply(0, S.last_s_out)
    return true
  end
  local starting = autotune.status == "idle"
  local max_error = yaw_max_error_param:get()
  local max_error_rad = math.rad(max_error)
  local hysteresis = starting and math.rad(yaw_max_angle_param:get()) or autotune.hyst_rad
  if not (hysteresis >= math.rad(1) and hysteresis <= math.rad(89) and
          max_error >= 5 and max_error <= 90 and hysteresis < max_error_rad) then
    gcs:send_text(MAV_SEVERITY.ERROR, "YawTune: require MAX_ANG < MAX_ANG_ERR (5-90)")
    S.done = true
    return false
  end
  if starting then
    local min_output = yaw_min_out_param:get()
    local max_output = yaw_max_out_param:get()
    if not (min_output >= 1 and min_output <= max_output and max_output <= 100) then
      gcs:send_text(MAV_SEVERITY.ERROR, "YawTune: require 1 <= MIN_OUT <= MAX_OUT <= 100")
      S.done = true
      return false
    end
    local max_cycles = yaw_max_cycle_param:get()
    if not (max_cycles >= 3 and max_cycles <= 60 and max_cycles % 1 == 0) then
      gcs:send_text(MAV_SEVERITY.ERROR, "YawTune: MAX_CYCLE must be an integer from 3 to 60")
      S.done = true
      return false
    end
    S.target_rad = ahrs:get_yaw()
    -- Start at MIN_OUT and step up toward MAX_OUT only while the rover does
    -- not turn, so the relay uses the smallest output that moves it
    autotune.relay = min_output / 100
    S.max_relay = max_output / 100
    S.still_s = 0
    S.still_lvl, S.move_lvl = nil, nil
    autotune.hyst_rad = hysteresis
    autotune.cycles = max_cycles - autotune.skip_cycles
    -- Each relay cycle sweeps about 4 x the switching angle. Ask for a turn
    -- rate that finishes all cycles in 60 % of the measuring time; just
    -- "moving" can be so slow that the run times out. The ramp's own time is
    -- added to the timeout so starting low costs no measuring time.
    S.ramp_rate = math.max(YAW_RATE_THRESH_RAD_S,
      4 * hysteresis * max_cycles / (0.6 * S.TUNE_TIMEOUT_S))
    autotune.timeout_s = S.TUNE_TIMEOUT_S +
      math.ceil((max_output - min_output) / 100 / S.RAMP_STEP) * S.RAMP_STILL_S
    autotune:start()
    S.last_progress = 0
    S.last_log_s = -1
    gcs:send_text(MAV_SEVERITY.NOTICE, string.format(
      "YawTune: started; target %.1f deg, max error %.1f deg, output %.0f-%.0f%%",
      math.deg(S.target_rad), max_error, min_output, max_output))
    gcs:send_text(MAV_SEVERITY.INFO, string.format("YawTune: relay switches at +/- %.1f deg", math.deg(hysteresis)))
  end
  local err = funcs:yawErrorRad(ahrs:get_yaw(), S.target_rad)
  local yaw_rate = ahrs:get_gyro():z()
  -- Output ramp: only before the first full relay cycle, so every measured
  -- cycle (the first two are discarded anyway) runs at one fixed amplitude
  if autotune.cycle == 0 then
    -- Breakaway: first output that really turns the rover (rate above 5 deg/s)
    if S.move_lvl == nil and math.abs(yaw_rate) >= S.MOVE_RATE and autotune.u ~= 0 then
      S.move_lvl = autotune.relay
    end
  end
  if autotune.cycle == 0 and autotune.relay < S.max_relay then
    if math.abs(yaw_rate) < S.ramp_rate then
      S.still_s = S.still_s + AUTOTUNE_PERIOD_MS / 1000.0
    else
      S.still_s = 0
    end
    if S.still_s >= S.RAMP_STILL_S then
      S.still_s = 0
      -- Last output at which the rover stayed still (under 2 deg/s for the whole second)
      if math.abs(yaw_rate) < YAW_RATE_THRESH_RAD_S and S.move_lvl == nil then
        S.still_lvl = autotune.relay
      end
      autotune.relay = math.min(S.max_relay, autotune.relay + S.RAMP_STEP)
      -- The relay holds its last output until the next switch; apply the new
      -- amplitude now, same direction
      if autotune.u ~= 0 then
        autotune.u = (autotune.u > 0) and autotune.relay or -autotune.relay
      end
      gcs:send_text(MAV_SEVERITY.INFO, string.format("YawTune: turn %.0f<%.0f deg/s; output %.0f%%",
        math.deg(math.abs(yaw_rate)), math.deg(S.ramp_rate), autotune.relay * 100))
    end
  end
  autotune.max_amp_rad = max_error_rad
  local s_out, status = autotune:step(err, AUTOTUNE_PERIOD_MS / 1000.0)
  S.last_s_out = s_out
  apply(0, s_out)
  if autotune.cycle > S.last_progress or autotune.t - S.last_log_s >= 2 then
    S.last_progress = autotune.cycle
    S.last_log_s = autotune.t
    gcs:send_text(MAV_SEVERITY.INFO, string.format(
      "YawTune: t %.0fs cycle %d/%d err %.1fdeg rate %.1fdeg/s out %.2f",
      autotune.t, autotune.cycle, autotune.skip_cycles + autotune.cycles,
      math.deg(err), math.deg(yaw_rate), s_out))
  end
  if status == "done" then
    local r = autotune:result()
    local gain_multiplier = YA.gain_multiplier()
    local effective_p = r.P * gain_multiplier
    local effective_i = r.I * gain_multiplier
    param:set_and_save('SCR_USER2', r.P)
    param:set_and_save('SCR_USER3', r.I)
    saveBreakaway()
    YA.pid:setGains(effective_p, effective_i, 0)
    YA.pid:resetInternalState()
    gcs:send_text(MAV_SEVERITY.NOTICE, string.format("YawTune: Ku=%.2f Pu=%.2fs a=%.1fdeg", r.Ku, r.Pu, math.deg(r.amp)))
    gcs:send_text(MAV_SEVERITY.NOTICE, string.format(
      "YawTune: saved base P=%.3f I=%.3f; using P=%.3f I=%.3f (x%.2f)",
      r.P, r.I, effective_p, effective_i, gain_multiplier))
    if param:get(TUNE_SELECT_PARAM) == TUNE_MODE.YAW then
      param:set_and_save(TUNE_SELECT_PARAM, TUNE_MODE.NORMAL)
      gcs:send_text(MAV_SEVERITY.NOTICE, "YawTune: SCR_USER1 set to 0 (normal)")
    end
    S.done = true
    apply(0, 0)
    YA.start_verification_sequence()
  elseif status == "fail" then
    gcs:send_text(MAV_SEVERITY.ERROR, "YawTune: failed - " .. autotune:result().msg)
    gcs:send_text(MAV_SEVERITY.WARNING, "YawTune: steering output stopped; selector remains available for retry")
    S.done = true
    apply(0, 0)
  end
  return true
end

--[[
Armed, every mixer cycle. The relay owns the motors only in HOLD; any other
mode keeps normal control, and after a failure or abort the pilot gets the
rover back.
-- @return bool - true when the tune ran this cycle (the mixer stops here)
--]]
function YT.handle(selector, vehicle_mode)
  if selector ~= TUNE_MODE.YAW and (autotune.status ~= "idle" or S.done) then
    if autotune.status == "running" then
      gcs:send_text(MAV_SEVERITY.WARNING, "YawTune: aborted (SCR_USER1 changed)")
    end
    autotune:reset()
    S.done = false
    S.last_tick_ms, S.last_s_out = 0, 0
  elseif selector == TUNE_MODE.YAW then
    if autotune.status == "running" and vehicle_mode ~= DRIVING_MODES.HOLD then
      autotune:reset()
      S.done = true
      S.last_tick_ms, S.last_s_out = 0, 0
      apply(0, 0)
      gcs:send_text(MAV_SEVERITY.WARNING, "YawTune: aborted (left HOLD); set SCR_USER1 to 0 then 2 to retry")
    elseif autotune.status == "running" or (not S.done and vehicle_mode == DRIVING_MODES.HOLD) then
      run()
      if autotune.status == "fail" or S.done then apply(0, 0) end
      return true
    elseif not S.done then
      local now_ms = millis():toint()
      if now_ms - S.hold_hint_ms >= 5000 then
        S.hold_hint_ms = now_ms
        gcs:send_text(MAV_SEVERITY.INFO, "YawTune: switch to HOLD to start")
      end
    end
  end
  return false
end

function YT.on_disarm()
  if autotune.status == "running" then
    gcs:send_text(MAV_SEVERITY.WARNING, "YawTune: aborted (disarmed)")
  end
end

function YT.on_disarm_reset()
  -- reset() allocates new tables; skip it when there is nothing to clear
  if autotune.status ~= "idle" then
    autotune:reset()
  end
  S.done = false
  S.last_tick_ms = 0
  S.last_s_out = 0
end

R.yaw_tune = YT
return YT
