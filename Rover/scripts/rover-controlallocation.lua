--@param control_output integer - CONTROLE MODE 4
--| '1' # Roll
--| '2' # Pitch
--| '3' # Throttle
--| '4' # Yaw
--| '5' # Lateral
--| '6' # MainSail
--| '7' # WingSail
--| '8' # Walking_Height
--@return number|nil
-- External modules
package.path = package.path .. ';./scripts/?.lua;./scripts/modules/?.lua;/scripts/modules/?.lua'
local PID = require("pid")
local funcs = require("functions")
local Autotune = require("autotune")
local CircleTune = require("circle-tune")
-------------------------------------------------------------------------------
------------------------- GLOBAL SCOPE DEFINITIONS ----------------------------
-------------------------------------------------------------------------------
-- Channel control variables
local THROTTLE_CONTROL_OUTPUT_CHANNEL = 3
local CONTROL_OUTPUT_YAW = 4
local THROTTLE_PARAM_TABLE_KEY = 17
local THROTTLE_PARAM_TABLE_PREFIX = 'THRO_'
assert(param:add_table(THROTTLE_PARAM_TABLE_KEY, THROTTLE_PARAM_TABLE_PREFIX, 3), 'could not add Lua throttle parameter table')
--[[
  // @Param: THRO_MAX_LUA
  // @DisplayName: Lua maximum motor output
  // @Description: Maximum motor PWM range as a percentage of the 1000-2000 us range around the 1500 us trim
  // @Units: %
  // @Range: 0 100
  // @Increment: 1
  // @User: Standard
--]]
assert(param:add_param(THROTTLE_PARAM_TABLE_KEY, 1, 'MAX_LUA', 90), 'could not add THRO_MAX_LUA parameter')
local throttle_max_lua_param = Parameter()
assert(throttle_max_lua_param:init('THRO_MAX_LUA'), 'could not bind THRO_MAX_LUA parameter')
--[[
  // @Param: THRO_MIN_LUA
  // @DisplayName: Lua minimum motor PWM
  // @Description: Minimum PWM above neutral to start a commanded yaw alignment from rest. Mirrored below neutral. 1500 disables it.
  // @Units: us
  // @Range: 1500 2000
  // @Increment: 1
  // @User: Standard
--]]
assert(param:add_param(THROTTLE_PARAM_TABLE_KEY, 2, 'MIN_LUA', 1500), 'could not add Lua minimum motor PWM parameter')
local throttle_min_lua_param = Parameter()
assert(throttle_min_lua_param:init('THRO_MIN_LUA'), 'could not bind THRO_MIN_LUA parameter')
-- CAN path (esc_can.lua). Lua scripts do not share globals, so the mixer and the
-- driver talk through RAM-only ESC_* parameters. Everything CAN-specific in this
-- file is behind can_selected(); with ESC_CAN = 0 the PWM path is unchanged.
local can_selected, pwm_allowed, sendCanCommand, modePeriod
do
  local ESC_STAT_CAN = 1
  -- Match esc_can reactivity: publish L/R as fast as scripting allows on the CAN path.
  local CAN_MIXER_PERIOD_MS = 20
  local esc_stat_param = Parameter()
  local esc_cmd_l_param = Parameter()
  local esc_cmd_r_param = Parameter()
  local esc_cmd_seq_param = Parameter()
  local esc_can_param = Parameter()
  local esc_params_bound = false
  local esc_bind_retry_ms = 0
  local esc_cmd_seq = 0

  local function bindEscParams()
    if esc_params_bound then
      return true
    end
    local now = millis():toint()
    if now < esc_bind_retry_ms then
      return false
    end
    esc_bind_retry_ms = now + 2000
    esc_params_bound = esc_stat_param:init('ESC_STAT') and esc_cmd_l_param:init('ESC_CMD_L') and
      esc_cmd_r_param:init('ESC_CMD_R') and esc_cmd_seq_param:init('ESC_CMD_SEQ') and
      esc_can_param:init('ESC_CAN')
    return esc_params_bound
  end

  can_selected = function()
    if not bindEscParams() then
      return false
    end
    return (math.floor(esc_stat_param:get() or 0) & ESC_STAT_CAN) ~= 0
  end

  -- Any pulse on MAIN 1..4 drops the C620 out of CAN mode, so with ESC_CAN = 1 the
  -- mixer must not write PWM even before esc_can.lua has published ESC_STAT
  -- esc_can.lua creates the ESC_ table when it loads, before this script's
  -- first update. No table means no CAN driver, so PWM is the only path.
  -- A bound Parameter avoids a by-name lookup of the whole tree every cycle.
  pwm_allowed = function()
    if not bindEscParams() then
      return true
    end
    return (esc_can_param:get() or 0) < 0.5
  end

  sendCanCommand = function(norm_l, norm_r)
    esc_cmd_l_param:set(norm_l)
    esc_cmd_r_param:set(norm_r)
    esc_cmd_seq = (esc_cmd_seq + 1) % 1000000
    esc_cmd_seq_param:set(esc_cmd_seq)
  end

  -- In CAN the mixer runs faster so the driver's interpolation adds little delay
  modePeriod = function(pwm_period_ms)
    if can_selected() then
      return math.min(pwm_period_ms, CAN_MIXER_PERIOD_MS)
    end
    return pwm_period_ms
  end

end

-- CAN gain set. ATC_* and CRUISE_THROTTLE hold the set of the active path;
-- the other path's copy lives here and is swapped in while disarmed.
-- Vehicle type control
local VEHICLE_TYPE = param:get('SCR_USER5')
-- TRIM values
local PWM_TRIM_VALUE = 1500
local RC1_TRIM_VALUE = param:get('RC1_TRIM')
local RC3_TRIM_VALUE = param:get('RC3_TRIM')
-- Main-chunk locals are capped at 100 in ArduPilot Lua. State added for the
-- CAN path and the loop timing lives in this one table.
local S = {
  -- Disarmed: poll at 200 ms so arming is picked up quickly (it was 2 s, and
  -- the first stick command after arming waited up to that long)
  DISARMED_PERIOD_MS = 200,
  DISARMED_SYNC_MS = 2000,
  last_gain_sync_ms = -2000,
  -- Read every cycle: bound objects instead of by-name lookups
  tune_select = Parameter(),
  rc3_reversed = Parameter(),
  -- Yaw autotune: HOLD hint rate limit and the start-output ramp
  yaw_hold_hint_ms = -5000,
  yaw_still_s = 0,
  yaw_max_relay = 1,
  yaw_ramp_rate = 0,
  YAW_RAMP_STILL_S = 1.0, -- time below the ramp turn rate before raising the output
  YAW_RAMP_STEP = 0.05,   -- raise by 5 % of the output range per step
  YAW_TUNE_TIMEOUT_S = 60, -- relay measuring time, as in modules/autotune.lua
  YAW_MOVE_RATE = math.rad(5), -- turn rate that counts as the rover really turning
  YAW_DEADBAND_CAN = 0.005,    -- 0.6 rpm at ESC_RPM 120: below the CAN stall-boost minimum  yaw_still_lvl = nil,     -- last ramp output that left the rover still
  yaw_move_lvl = nil,      -- first ramp output that turned it
}
assert(S.tune_select:init('SCR_USER1'), 'could not bind SCR_USER1 parameter')
assert(S.rc3_reversed:init('RC3_REVERSED'), 'could not bind RC3_REVERSED parameter')
--[[
  // @Param: THRO_INV
  // @DisplayName: Invert ArduPilot throttle
  // @Description: 1 reverses the throttle that ArduPilot's own modes send (Auto, Circle, Acro, RTL, Simple, Circle tune) before the wheels. The manual stick path is never changed: it already takes forward as (trim - stick), the opposite of ArduPilot's throttle sign, so with 1 forward means forward in every mode. 0 keeps ArduPilot's sign.
  // @Values: 0:ArduPilot sign,1:Inverted (match the manual stick)
  // @User: Standard
--]]
assert(param:add_param(THROTTLE_PARAM_TABLE_KEY, 3, 'INV', 1), 'could not add THRO_INV parameter')
S.thro_inv = Parameter()
assert(S.thro_inv:init('THRO_INV'), 'could not bind THRO_INV parameter')

-- Throttle from ArduPilot's controllers in the sign the wheels use. On this
-- rover forward is negative for the motors (manual forward logged -109 rpm on
-- all four wheels), and ArduPilot's positive throttle drove the rover backward
-- in Circle and Auto. Steering already matches and is not touched.
function S.apThrottle(t)
  t = tonumber(t) or 0
  if S.thro_inv:get() >= 0.5 then
    return -t
  end
  return t
end

-- Stick deadzone for the CAN manual path: RCn_DZ when set, otherwise 20 us
function S.stickDeadzone(name)
  local dz = param:get(name) or 0
  if dz > 0 then
    return math.min(dz, 200)
  end
  return 20
end
S.RC1_DZ_US = S.stickDeadzone('RC1_DZ')
S.RC3_DZ_US = S.stickDeadzone('RC3_DZ')

-- Normalised stick (-1..1 at +/-450 us) with the deadzone removed and the
-- rest of the travel rescaled, so full stick still gives full command
function S.stickNorm(pwm, trim, dz_us)
  local delta = pwm - trim
  if math.abs(delta) <= dz_us then
    return 0
  end
  delta = (delta > 0) and (delta - dz_us) or (delta + dz_us)
  return delta / (450 - dz_us)
end
-- Signal smoothing logic
local last_manual_throttle = 0
local throttle_accel_rate_thresh = 0.5
local throttle_accel_rate = 0.5
local last_manual_steering = 0
local steering_accel_rate_thresh = 0.6
local steering_accel_rate = 0.6
-- Yaw alignment logic
local UPDATE_PERIOD_MS = 200
local UPDATE_DT = UPDATE_PERIOD_MS / 1000.0
local YAW_THRESH_RAD = math.rad(2)
local YAW_RATE_THRESH_RAD_S = math.rad(2)
local YAW_SETTLE_STEPS = 3 -- require 600 ms continuously within angle and rate limits
local YAW_DEADBAND = 0.02
local YAW_ALIGN_TIMEOUT_MS = 15000
local YAW_ALIGN_LOG_INTERVAL_MS = 1000
local last_yaw_align_log_ms = 0
local YAW_SEQUENCE_DEGREES = { 0, 90, 180, 270, 360 }
local YAW_SEQUENCE_PAUSE_MS = 5000
local YAW_SEQUENCE_MOVEMENT_RAD = math.rad(1)
local YAW_SEQUENCE_DEFAULT_TIMEOUT_S = 15
local yaw_sequence_active = false
local yaw_sequence_index = 0
local yaw_sequence_pause_start = nil
local yaw_sequence_last_movement_ms
local yaw_sequence_movement_reference_rad
local yaw_sequence_timeout_ms
-- Params: p_gain, i_gain, d_gain, i_max, pid_max
local steering_steady_pid = PID:new(3.5, 8, 0, 1, 0.95)
local yaw_target_rad = nil
local yaw_align_steps = 0
local yaw_settle_steps = 0
-- In CAN the mixer runs every 20 ms but the yaw PI and relay keep their 200 ms step
local YAW_TICK_TOLERANCE_MS = 10
local yaw_last_tick_ms = 0
local yaw_last_s_out = 0
local yaw_tune_last_tick_ms = 0
local yaw_tune_last_s_out = 0

local last_nav_idx = nil

local radio_type = 0

-- Tuning selector parameter (SCR_USER1):
--   0 = normal operation
--   1 = Pixhawk PID tune (Rover QuickTune) - starts while armed
--   2 = yaw PI autotune (relay)            - starts armed in HOLD
--   3 = Circle steering-rate PI (relay)    - starts while armed in Circle
-- Yaw autotune saves SCR_USER2 (P) and SCR_USER3 (I).
local TUNE_SELECT_PARAM = 'SCR_USER1'
local TUNE_MODE = { NORMAL = 0, PIXHAWK = 1, YAW = 2, CIRCLE = 3 }
local circle_tune = CircleTune:new()
local AUTOTUNE_PERIOD_MS = UPDATE_PERIOD_MS

-- Yaw autotune parameters
local YAW_TUNE_PARAM_TABLE_KEY = 16
assert(param:add_table(YAW_TUNE_PARAM_TABLE_KEY, 'YAWT_', 7), 'could not add yaw autotune parameter table')
-- Keep the saved abort limit and switching angle in their original slots.
assert(param:add_param(YAW_TUNE_PARAM_TABLE_KEY, 1, 'MAX_ANG_ERR', 90), 'could not add yaw autotune error limit')
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
assert(param:add_param(YAW_TUNE_PARAM_TABLE_KEY, 2, 'MAX_OUT', 100), 'could not add yaw autotune max output parameter')
local yaw_max_out_param = Parameter()
assert(yaw_max_out_param:init('YAWT_MAX_OUT'), 'could not bind YAWT_MAX_OUT parameter')

--[[
  // @Param: YAWT_GAIN_MULT
  // @DisplayName: Yaw autotune gain multiplier
  // @Description: Multiplier applied to tuned P and I gains when aligning yaw
  // @Range: 0.1 10000
  // @Increment: 0.1
  // @User: Standard
--]]
assert(param:add_param(YAW_TUNE_PARAM_TABLE_KEY, 3, 'GAIN_MULT', 1), 'could not add yaw autotune gain multiplier parameter')
local yaw_gain_multiplier_param = Parameter()
assert(yaw_gain_multiplier_param:init('YAWT_GAIN_MULT'), 'could not bind YAWT_GAIN_MULT parameter')

--[[
  // @Param: YAWT_MAX_ANG
  // @DisplayName: Yaw autotune relay switching angle
  // @Description: Heading error on either side of the starting heading where the relay reverses. Must be below YAWT_MAX_ANG_ERR.
  // @Units: deg
  // @Range: 1 89
  // @User: Standard
--]]
assert(param:add_param(YAW_TUNE_PARAM_TABLE_KEY, 4, 'MAX_ANG', 45), 'could not add yaw autotune switching angle parameter')
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
assert(param:add_param(YAW_TUNE_PARAM_TABLE_KEY, 5, 'MIN_OUT', 5), 'could not add yaw autotune min output parameter')
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
assert(param:add_param(YAW_TUNE_PARAM_TABLE_KEY, 6, 'MAX_CYCLE', 6), 'could not add yaw autotune cycle parameter')
local yaw_max_cycle_param = Parameter()
assert(yaw_max_cycle_param:init('YAWT_MAX_CYCLE'), 'could not bind YAWT_MAX_CYCLE parameter')

--[[
  // @Param: YAWT_TIMEOUT
  // @DisplayName: Yaw verification inactivity timeout
  // @Description: Time without at least one degree of heading change before stopping the post-tune yaw verification
  // @Units: s
  // @Range: 1 120
  // @User: Standard
--]]
assert(param:add_param(YAW_TUNE_PARAM_TABLE_KEY, 7, 'TIMEOUT', YAW_SEQUENCE_DEFAULT_TIMEOUT_S),
  'could not add yaw verification timeout parameter')
local yaw_timeout_param = Parameter()
assert(yaw_timeout_param:init('YAWT_TIMEOUT'), 'could not bind YAWT_TIMEOUT parameter')

local function getYawGainMultiplier()
  return math.max(0.1, math.min(10000, yaw_gain_multiplier_param:get()))
end

local autotune = Autotune:new({ relay = 0.20 })
local autotune_target_rad = nil
local autotune_done = false
local yaw_tune_last_progress = 0
local yaw_tune_last_log_s = 0
local selector_armed_reported = false

-- Load tuned gains if present (0 means "not tuned yet", keep defaults)
local saved_p, saved_i = param:get('SCR_USER2') or 0, param:get('SCR_USER3') or 0
if saved_p > 0 and saved_i > 0 then
  local gain_multiplier = getYawGainMultiplier()
  steering_steady_pid:setGains(saved_p * gain_multiplier, saved_i * gain_multiplier, 0)
end

local syncGainSet
do
  -- Preserve the saved slots of the CAN ATC gain banks. Slots 1-3 were yaw settings.
  local CAN_GAINS_PARAM_TABLE_KEY = 19
  local CAN_GAIN_SWAP = {
    { 'ATC_STR_RAT_FF', 'STRFF' },
    { 'ATC_STR_RAT_P', 'STRP' },
    { 'ATC_STR_RAT_I', 'STRI' },
    { 'ATC_SPEED_P', 'SPDP' },
    { 'ATC_SPEED_I', 'SPDI' },
    { 'CRUISE_THROTTLE', 'CRUISE' },
  }
  assert(param:add_table(CAN_GAINS_PARAM_TABLE_KEY, 'CANT_', 5 + 2 * #CAN_GAIN_SWAP), 'could not add CANT_ parameter table')
  assert(param:add_param(CAN_GAINS_PARAM_TABLE_KEY, 4, 'ACTIVE', 0), 'could not add CANT_ACTIVE parameter')
  assert(param:add_param(CAN_GAINS_PARAM_TABLE_KEY, 5, 'C_VALID', 0), 'could not add CANT_C_VALID parameter')
  for n, item in ipairs(CAN_GAIN_SWAP) do
    assert(param:add_param(CAN_GAINS_PARAM_TABLE_KEY, 5 + n, 'C_' .. item[2], 0), 'could not add CANT_C_' .. item[2])
  end
  for n, item in ipairs(CAN_GAIN_SWAP) do
    assert(param:add_param(CAN_GAINS_PARAM_TABLE_KEY, 5 + #CAN_GAIN_SWAP + n, 'P_' .. item[2], 0), 'could not add CANT_P_' .. item[2])
  end
  local function setAndSaveIfChanged(name, value)
    if value ~= nil and param:get(name) ~= value then
      param:set_and_save(name, value)
    end
  end

  --[[
  Keep ATC_* / CRUISE_THROTTLE on the gain set of the active output path.
  Only called while disarmed. With PWM only (CANT_ACTIVE = 0 and CAN never on)
  nothing is written.
  --]]
  syncGainSet = function()
    -- Give esc_can.lua time to publish ESC_STAT after boot before deciding the path
    if millis():toint() < 6000 then
      return
    end
    local can = can_selected()
    local want = can and 1 or 0
    local active = param:get('CANT_ACTIVE') or 0
    if want == active then
      if can then
        -- QuickTune and the Circle relay write ATC_*; keep the CAN copy current
        for _, item in ipairs(CAN_GAIN_SWAP) do
          setAndSaveIfChanged('CANT_C_' .. item[2], param:get(item[1]))
        end
      end
    else
      local out_prefix = (active == 1) and 'CANT_C_' or 'CANT_P_'
      for _, item in ipairs(CAN_GAIN_SWAP) do
        setAndSaveIfChanged(out_prefix .. item[2], param:get(item[1]))
      end
      if can and (param:get('CANT_C_VALID') or 0) == 0 then
        for _, item in ipairs(CAN_GAIN_SWAP) do
          setAndSaveIfChanged('CANT_C_' .. item[2], param:get(item[1]))
        end
        param:set_and_save('CANT_C_VALID', 1)
        gcs:send_text(MAV_SEVERITY.WARNING, 'CAN gains: first CAN use, starting from the PWM gains; retune in CAN')
      else
        local in_prefix = can and 'CANT_C_' or 'CANT_P_'
        for _, item in ipairs(CAN_GAIN_SWAP) do
          setAndSaveIfChanged(item[1], param:get(in_prefix .. item[2]))
        end
      end
      param:set_and_save('CANT_ACTIVE', want)
      gcs:send_text(MAV_SEVERITY.NOTICE, string.format('Gain set: %s loaded into ATC_*', can and 'CAN' or 'PWM'))
    end
  end

end

-- Severity for logging in GCS
MAV_SEVERITY = { EMERGENCY = 0, ALERT = 1, CRITICAL = 2, ERROR = 3, WARNING = 4, NOTICE = 5, INFO = 6, DEBUG = 7 }
-- Rover driving modes
DRIVING_MODES = { MANUAL = 0, ACRO = 1, STEERING = 3, HOLD = 4, LOITER = 5, SIMPLE = 7, CIRCLE = 9, AUTO = 10, RTL = 11, SMART_RTL = 12, GUIDED = 15 }
-- Mission states dictionary
MISSION_STATE = { IDLE = 0, RUNNING = 1, FINISHED = 2 }

-------------------------------------------------------------------------------
------------------------- LOW LEVEL ACTION FUNCTIONS --------------------------
-------------------------------------------------------------------------------
--[[
Control allocation
This function is responsible for the PWM signal allocation of the vehicle.
It receives the throttle (t) and steering (s) values and calculates the PWM values for the motors.
The function also takes into account the trim values for the PWM outputs.
-- @param t number - Throttle command from -1.0 to 1.0
-- @param s number - Steering command from -1.0 to 1.0
--]]
local function getLuaPWMRange()
  return 5 * math.max(0, math.min(100, throttle_max_lua_param:get()))
end

local function applyControlAllocation(t, s, timeout_ms)
  timeout_ms = timeout_ms or 300
  -- We assign the PWM values to the motors, which are opposite in sign for each diagonal pair
  -- MOTOR SCHEMATIC IN ROVER FRAME
  -- 1 - 0     ^
  --   |       | Rover forward direction
  -- 2 - 3
  -- Setting the PWM outputs based on the control allocation directions
  -- left for motors in the left side (1 and 2), right for the ones on the right side (0 and 3)
  local pwm_range = getLuaPWMRange()
  local pwm_aloc_l, pwm_aloc_r = funcs:allocateRightAndLeftPwmShare(t, s, pwm_range)
  local pwm_min = PWM_TRIM_VALUE - pwm_range
  local pwm_max = PWM_TRIM_VALUE + pwm_range
  local pwm_l = math.floor(funcs:mapMaxMin(PWM_TRIM_VALUE + pwm_aloc_l, pwm_min, pwm_max))
  local pwm_r = math.floor(funcs:mapMaxMin(PWM_TRIM_VALUE - pwm_aloc_r, pwm_min, pwm_max))
  if can_selected() then
    -- Skid-steer sum, same as the PWM servo mix: left = throttle + steering,
    -- right = throttle - steering. Scale both sides together when the sum
    -- passes 1 so a turn while moving does not drop one stick.
    -- The Circle relay uses this same mix, so the gains it identifies match
    -- the plant seen in normal CAN driving.
    local left, right = t + s, t - s
    local peak = math.max(1, math.abs(left), math.abs(right))
    sendCanCommand(left / peak, right / peak)
    return pwm_l, pwm_r
  end
  if not pwm_allowed() then
    return pwm_l, pwm_r
  end
  SRV_Channels:set_output_pwm_chan_timeout(0, pwm_r, timeout_ms)
  SRV_Channels:set_output_pwm_chan_timeout(1, pwm_l, timeout_ms)
  SRV_Channels:set_output_pwm_chan_timeout(2, pwm_l, timeout_ms)
  SRV_Channels:set_output_pwm_chan_timeout(3, pwm_r, timeout_ms)
  return pwm_l, pwm_r
end

-------------------------------------------------------------------------------
--------------------- YAW CONTROL VIA MAV_CMD_NAV_WAYPOINT --------------------
-------------------------------------------------------------------------------
--[[
Reset the yaw control state
--]]
local function resetYawControlState()
  yaw_target_rad = nil
  yaw_sequence_pause_start = nil
  yaw_align_steps = 0
  yaw_settle_steps = 0
  steering_steady_pid:resetInternalState()
  yaw_last_s_out = 0
  yaw_last_tick_ms = 0
end

-- True when the 200 ms yaw step is due; always true in PWM, where the mixer itself runs at 200 ms
local function yawTickDue(last_tick_ms)
  if not can_selected() then
    return true, last_tick_ms
  end
  local now = millis():toint()
  if last_tick_ms == 0 or now - last_tick_ms >= UPDATE_PERIOD_MS - YAW_TICK_TOLERANCE_MS then
    return true, now
  end
  return false, last_tick_ms
end

local function startYawVerificationHeading()
  resetYawControlState()
  yaw_target_rad = math.rad(YAW_SEQUENCE_DEGREES[yaw_sequence_index])
  yaw_sequence_movement_reference_rad = ahrs:get_yaw()
  yaw_sequence_last_movement_ms = millis()
  local timeout_s = yaw_timeout_param:get()
  if not (timeout_s >= 1 and timeout_s <= 120) then
    timeout_s = YAW_SEQUENCE_DEFAULT_TIMEOUT_S
  end
  yaw_sequence_timeout_ms = timeout_s * 1000
  last_yaw_align_log_ms = 0
  gcs:send_text(MAV_SEVERITY.INFO, string.format(
    'YawTune: tentando orientar para %d° (etapa %d/%d)',
    YAW_SEQUENCE_DEGREES[yaw_sequence_index], yaw_sequence_index, #YAW_SEQUENCE_DEGREES))
end

local function startYawVerificationSequence()
  yaw_sequence_active = true
  yaw_sequence_index = 1
  gcs:send_text(MAV_SEVERITY.NOTICE, 'YawTune: tentando orientar: 0°, 90°, 180°')
  gcs:send_text(MAV_SEVERITY.NOTICE, 'YawTune: depois 270° e 360°/0°')
  startYawVerificationHeading()
  if not vehicle:set_mode(DRIVING_MODES.STEERING) then
    yaw_sequence_active = false
    resetYawControlState()
    gcs:send_text(MAV_SEVERITY.ERROR, 'YawTune: nao foi possivel entrar em STEERING; sequencia cancelada')
    applyControlAllocation(0, 0)
  end
end

local function advanceYawVerificationSequence()
  local completed_heading = YAW_SEQUENCE_DEGREES[yaw_sequence_index]
  gcs:send_text(MAV_SEVERITY.NOTICE, string.format(
    'YawTune: orientacao para %d° concluida', completed_heading))
  if yaw_sequence_index >= #YAW_SEQUENCE_DEGREES then
    yaw_sequence_active = false
    yaw_sequence_index = 0
    resetYawControlState()
    applyControlAllocation(0, 0)
    vehicle:set_mode(DRIVING_MODES.HOLD)
    gcs:send_text(MAV_SEVERITY.NOTICE, 'YawTune: sequencia concluida; rover em HOLD')
    return
  end

  resetYawControlState()
  yaw_sequence_pause_start = millis()
  gcs:send_text(MAV_SEVERITY.INFO, "YawTune: aguardando 5 s")
end

--[[
Decode yaw from waypoint z (altitude) field
Encoding:
  -1: pass-through waypoint
  Any other value: align yaw to the angle informed in the waypoint altitude
-- @param angle_from_alt number
-- @return number|nil, bool
--]]
local function decodeYawFromWaypointZ(angle_from_alt)
  if angle_from_alt == nil or funcs:isNan(angle_from_alt) then
    return nil, false
  end

  if angle_from_alt == -1 then
    return nil, true
  end

  return funcs:mapTo360(angle_from_alt), false
end

--[[
Check if a waypoint was reached and trigger yaw control if param4 is valid
-- @return bool - true if yaw control was triggered
--]]
local function triggerYawControlOnReachedWaypoint()
  local idx = mission:get_current_nav_index()
  if not idx then
    return false
  end

  -- Initialize the index tracker
  if last_nav_idx == nil then
    last_nav_idx = idx
    return false
  end

  -- When the nav index changes, the previous waypoint was reached
  if idx ~= last_nav_idx then
    local reached_idx = last_nav_idx
    last_nav_idx = idx
    local item = mission:get_item(reached_idx)
    if not item then
      resetYawControlState()
      return false
    end

    -- Only handle NAV_WAYPOINT (16) with valid param4 (yaw)
    if item:command() ~= 16 then
      resetYawControlState()
      return false
    end

    local yaw_target_deg, is_pass_through = decodeYawFromWaypointZ(item:z())

    -- If angle == -1, treat as pass-through waypoint: do NOT switch modes
    -- This value must be used in the waypoint before a DO_SET_REVERSE transition
    -- so the mission remains in AUTO while ArduRover changes the driving direction.
    if is_pass_through then
      -- clear any previous target just in case
      resetYawControlState()
      return false
    end

    if yaw_target_deg == nil then
      resetYawControlState()
      return false
    end

    yaw_target_rad = math.rad(yaw_target_deg)

    -- Reset PID state and start alignment
    steering_steady_pid:resetInternalState()
    yaw_align_steps = 0
    yaw_settle_steps = 0
    yaw_last_s_out = 0
    yaw_last_tick_ms = 0
    -- Stop the vehicle and take over yaw using STEERING mode
    applyControlAllocation(0, 0)
    vehicle:set_mode(DRIVING_MODES.STEERING)
    return true
  end
  return false
end

-------------------------------------------------------------------------------
------------------------- HIGH LEVEL CONTROL FUNCTIONS ------------------------
-------------------------------------------------------------------------------
--[[
Control the actions while not armed
--]]
local function notArmed()
  if can_selected() then
    -- A trim pulse on MAIN 1..4 knocks the C620 off CAN; esc_can.lua holds them at 0
    sendCanCommand(0, 0)
    last_manual_throttle = 0
    last_manual_steering = 0
    return
  end
  if not pwm_allowed() then
    return
  end
  SRV_Channels:set_output_pwm_chan_timeout(0, PWM_TRIM_VALUE, 3000)
  SRV_Channels:set_output_pwm_chan_timeout(1, PWM_TRIM_VALUE, 3000)
  SRV_Channels:set_output_pwm_chan_timeout(2, PWM_TRIM_VALUE, 3000)
  SRV_Channels:set_output_pwm_chan_timeout(3, PWM_TRIM_VALUE, 3000)
end

--[[
Perform vehicle control in Manual mode
--]]
local function applyPWMManualMode()
  -- get_pwm may return nil; treat that as a centred stick, not a script error
  local rc3_pwm = rc:get_pwm(3) or RC3_TRIM_VALUE
  local rc1_pwm = rc:get_pwm(1) or RC1_TRIM_VALUE
  local raw_throttle = 0

  if can_selected() then
    -- CAN path is fully reactive: no Lua stick smoothing. The C620 PWM input
    -- has its own neutral band; the CAN speed loop does not, so a few us of
    -- stick jitter around trim became a small rpm command that made the
    -- wheels creep and shake. Apply the deadzone here.
    local throttle_sign = (radio_type == 1) and 1 or -1
    applyControlAllocation(S.stickNorm(rc3_pwm, RC3_TRIM_VALUE, S.RC3_DZ_US) * throttle_sign,
      S.stickNorm(rc1_pwm, RC1_TRIM_VALUE, S.RC1_DZ_US))
    return
  end

  -- Compares the diff from the last manual signals to the maximum rate we are accepting
  -- Make the actual command be a rate from the last to the required command if necessary
  local raw_steering = (rc1_pwm - RC1_TRIM_VALUE) / 450
  local steering = funcs:applyAbsSmoothing(raw_steering, last_manual_steering, steering_accel_rate_thresh,
    steering_accel_rate)
  last_manual_steering = steering
  if radio_type == 1 then
    raw_throttle = (rc3_pwm - RC3_TRIM_VALUE) / 450
  else
    raw_throttle = (RC3_TRIM_VALUE - rc3_pwm) / 450
  end
  local throttle = funcs:applyAbsSmoothing(raw_throttle, last_manual_throttle, throttle_accel_rate_thresh,
    throttle_accel_rate)
  last_manual_throttle = throttle

  applyControlAllocation(throttle, steering)
end

--[[
Perform vehicle control in Steering mode
--]]
local function applyPWMSteeringMode()
  -- If the pilot or failsafe switched modes, stop pursuing yaw alignment
  if vehicle:get_mode() ~= DRIVING_MODES.STEERING then
    steering_steady_pid:resetInternalState()
    yaw_settle_steps = 0
    return
  end

  if yaw_sequence_pause_start then
    if (millis() - yaw_sequence_pause_start):tofloat() < YAW_SEQUENCE_PAUSE_MS then
      applyControlAllocation(0, 0)
      return
    end
    yaw_sequence_index = yaw_sequence_index + 1
    startYawVerificationHeading()
  end
  if yaw_target_rad == nil then
    applyControlAllocation(0, 0)
    return
  end

  local tick_due
  tick_due, yaw_last_tick_ms = yawTickDue(yaw_last_tick_ms)
  if not tick_due then
    applyControlAllocation(0, yaw_last_s_out)
    return
  end

  local current_yaw_rad = ahrs:get_yaw()
  local now = millis()
  if yaw_sequence_active then
    local movement = funcs:yawErrorRad(current_yaw_rad, yaw_sequence_movement_reference_rad)
    if math.abs(movement) >= YAW_SEQUENCE_MOVEMENT_RAD then
      yaw_sequence_movement_reference_rad = current_yaw_rad
      yaw_sequence_last_movement_ms = now
    elseif (now - yaw_sequence_last_movement_ms):tofloat() >= yaw_sequence_timeout_ms then
      local failed_heading = YAW_SEQUENCE_DEGREES[yaw_sequence_index]
      yaw_sequence_active = false
      yaw_sequence_index = 0
      resetYawControlState()
      applyControlAllocation(0, 0)
      vehicle:set_mode(DRIVING_MODES.HOLD)
      gcs:send_text(MAV_SEVERITY.ERROR, string.format(
        'YawTune: %.1fs without movement at %d deg; HOLD', yaw_sequence_timeout_ms / 1000, failed_heading))
      return
    end
  elseif yaw_align_steps >= math.floor(YAW_ALIGN_TIMEOUT_MS / UPDATE_PERIOD_MS) then
    applyControlAllocation(0, 0)
    resetYawControlState()
    vehicle:set_mode(DRIVING_MODES.AUTO)
    return
  else
    yaw_align_steps = yaw_align_steps + 1
  end

  local err = funcs:yawErrorRad(current_yaw_rad, yaw_target_rad)
  local now_ms = now:toint()
  if now_ms - last_yaw_align_log_ms >= YAW_ALIGN_LOG_INTERVAL_MS then
    last_yaw_align_log_ms = now_ms
    gcs:send_text(MAV_SEVERITY.INFO, string.format(
      "YawAlign target=%.1f current=%.1f error=%.1f",
      math.deg(yaw_target_rad), math.deg(current_yaw_rad), math.deg(err)))
  end

  -- Require both heading and yaw rate to settle before stopping. This avoids
  -- switching to HOLD while the rover is still rotating through the target.
  local yaw_rate = ahrs:get_gyro():z()
  if math.abs(err) <= YAW_THRESH_RAD and math.abs(yaw_rate) <= YAW_RATE_THRESH_RAD_S then
    yaw_settle_steps = yaw_settle_steps + 1
  else
    yaw_settle_steps = 0
  end

  if yaw_settle_steps >= YAW_SETTLE_STEPS then
    applyControlAllocation(0, 0)
    steering_steady_pid:resetInternalState()
    yaw_settle_steps = 0
    if yaw_sequence_active then
      advanceYawVerificationSequence()
    else
      -- Set HOLD mode so the vehicle stops for the external ROS photo routine
      resetYawControlState()
      vehicle:set_mode(DRIVING_MODES.HOLD)
    end
    return
  end

  -- Rotate in place with pid output.
  -- Clear the integrator once the error changes sign (target passed): on a
  -- long turn it wound up, and after the overshoot it cancelled the P term
  -- for seconds (Ti about 12 s), leaving a 10 degree error with a zero
  -- command (seen on CAN: target 0 rpm for 7 s at -11 to -9 degrees).
  if steering_steady_pid.integrator * err < 0 then
    steering_steady_pid.integrator = 0
  end
  local s_out = steering_steady_pid:compute(err, UPDATE_DT)
  -- PWM only: the ESC gets a raw duty, so a stalled rover needs a minimum kick.
  -- On CAN the command is a wheel speed target and esc_can.lua breaks the
  -- wheels free itself (ESC_STALL), so no kick there.
  if not can_selected() and math.abs(err) > YAW_THRESH_RAD and math.abs(yaw_rate) <= YAW_RATE_THRESH_RAD_S
      and s_out * err > 0 then
    local pwm_range = getLuaPWMRange()
    local min_offset = throttle_min_lua_param:get() - PWM_TRIM_VALUE
    if pwm_range > 0 and min_offset > 0 then
      local min_command = math.min(1, min_offset / pwm_range)
      s_out = s_out < 0 and math.min(s_out, -min_command) or math.max(s_out, min_command)
    end
  end
  -- The deadband keeps a PWM ESC from chattering. On CAN the command is a wheel
  -- speed target that esc_can.lua can follow down to about 1 rpm, and 0.02
  -- (2.4 rpm) zeroed every command for heading errors under about 6 degrees.
  if math.abs(s_out) < (can_selected() and S.YAW_DEADBAND_CAN or YAW_DEADBAND) then
    s_out = 0
  end
  yaw_last_s_out = s_out
  applyControlAllocation(0, s_out)
end

--[[
Perform vehicle control in Auto mode
--]]
local function applyPWMAutoMode()
  local idx = mission:get_current_nav_index()

  -- Detect mission restart / rewind: current index went backwards
  if idx and last_nav_idx and idx < last_nav_idx then
    resetYawControlState()
    last_nav_idx = nil
  end

  -- Controls end of mission
  local mission_state = mission:state()
  if mission_state == MISSION_STATE.FINISHED then
    applyControlAllocation(0, 0)
    vehicle:set_mode(DRIVING_MODES.MANUAL)
    return
  end

  -- If we reached a waypoint, check if we need to align yaw from param4 with a valid value
  if triggerYawControlOnReachedWaypoint() then
    return
  end

  -- Acquiring throttle and steering from internal control output
  -- The throttle sign is preserved so MAV_CMD_DO_SET_REVERSE can be handled natively by ArduRover.
  local throttle = S.apThrottle(vehicle:get_control_output(THROTTLE_CONTROL_OUTPUT_CHANNEL))
  local steering = tonumber(vehicle:get_control_output(CONTROL_OUTPUT_YAW)) or 0
  applyControlAllocation(throttle, steering)
end

--[[
Pass the ArduRover internal controllers' outputs straight to the allocation
(used by CIRCLE/ACRO/RTL, e.g. during Rover QuickTune)
--]]
local function applyPWMInternalControl(timeout_ms)
  local throttle = S.apThrottle(vehicle:get_control_output(THROTTLE_CONTROL_OUTPUT_CHANNEL))
  local steering = tonumber(vehicle:get_control_output(CONTROL_OUTPUT_YAW)) or 0
  applyControlAllocation(throttle, steering, timeout_ms)
end

--[[
Save the measured turn breakaway as THRO_MIN_LUA. The yaw PI output for a
small heading error is far below the output that starts the rover turning
(about 35-40% on the ground), so without this the last degrees are left to
the slow integrator. THRO_MIN_LUA is the kick used when the rover is stalled
off target. The value is midway between the last ramp output that left the
rover still and the first one that turned it.
--]]
function S.saveYawBreakaway()
  if can_selected() then
    -- On CAN the wheel speed loop breaks the rover free (ESC_STALL); the PWM
    -- kick would only add a speed jump
    gcs:send_text(MAV_SEVERITY.INFO, "YawTune: CAN path, breakaway left to ESC_STALL; THRO_MIN_LUA kept")
    return
  end
  local still, move = S.yaw_still_lvl, S.yaw_move_lvl
  if move == nil or still == nil then
    gcs:send_text(MAV_SEVERITY.INFO, "YawTune: breakaway not seen (turned at the first output); THRO_MIN_LUA kept")
    return
  end
  local lvl = (still + move) / 2
  local pwm = math.floor(PWM_TRIM_VALUE + lvl * getLuaPWMRange() + 0.5)
  pwm = math.max(PWM_TRIM_VALUE, math.min(2000, pwm))
  param:set_and_save('THRO_MIN_LUA', pwm)
  gcs:send_text(MAV_SEVERITY.NOTICE, string.format("YawTune: breakaway %.0f%% (still %.0f, turns %.0f); THRO_MIN_LUA=%d",
    lvl * 100, still * 100, move * 100, pwm))
end

--[[
Yaw PI autotune in HOLD mode.
Switch high starts the relay test around the current yaw; switch low aborts/rearms.
-- @return bool - true while the autotune owns the outputs
--]]
local function runYawAutotune()
  if autotune_done then return false end
  local tick_due
  tick_due, yaw_tune_last_tick_ms = yawTickDue(yaw_tune_last_tick_ms)
  if not tick_due then
    applyControlAllocation(0, yaw_tune_last_s_out)
    return true
  end
  local starting = autotune.status == "idle"
  local max_error = yaw_max_error_param:get()
  local max_error_rad = math.rad(max_error)
  local hysteresis = starting and math.rad(yaw_max_angle_param:get()) or autotune.hyst_rad
  if not (hysteresis >= math.rad(1) and hysteresis <= math.rad(89) and
          max_error >= 5 and max_error <= 90 and hysteresis < max_error_rad) then
    gcs:send_text(MAV_SEVERITY.ERROR, "YawTune: require MAX_ANG < MAX_ANG_ERR (5-90)")
    autotune_done = true
    return false
  end
  if starting then
    local min_output = yaw_min_out_param:get()
    local max_output = yaw_max_out_param:get()
    if not (min_output >= 1 and min_output <= max_output and max_output <= 100) then
      gcs:send_text(MAV_SEVERITY.ERROR, "YawTune: require 1 <= MIN_OUT <= MAX_OUT <= 100")
      autotune_done = true
      return false
    end
    local max_cycles = yaw_max_cycle_param:get()
    if not (max_cycles >= 3 and max_cycles <= 60 and max_cycles % 1 == 0) then
      gcs:send_text(MAV_SEVERITY.ERROR, "YawTune: MAX_CYCLE must be an integer from 3 to 60")
      autotune_done = true
      return false
    end
    autotune_target_rad = ahrs:get_yaw()
    -- Start at MIN_OUT and step up toward MAX_OUT only while the rover does
    -- not turn, so the relay uses the smallest output that moves it
    autotune.relay = min_output / 100
    S.yaw_max_relay = max_output / 100
    S.yaw_still_s = 0
    S.yaw_still_lvl, S.yaw_move_lvl = nil, nil
    autotune.hyst_rad = hysteresis
    autotune.cycles = max_cycles - autotune.skip_cycles
    -- Each relay cycle sweeps about 4 x the switching angle. Ask for a turn
    -- rate that finishes all cycles in 60 % of the measuring time; just
    -- "moving" can be so slow that the run times out. The ramp's own time is
    -- added to the timeout so starting low costs no measuring time.
    S.yaw_ramp_rate = math.max(YAW_RATE_THRESH_RAD_S,
      4 * hysteresis * max_cycles / (0.6 * S.YAW_TUNE_TIMEOUT_S))
    autotune.timeout_s = S.YAW_TUNE_TIMEOUT_S +
      math.ceil((max_output - min_output) / 100 / S.YAW_RAMP_STEP) * S.YAW_RAMP_STILL_S
    autotune:start()
    yaw_tune_last_progress = 0
    yaw_tune_last_log_s = -1
    gcs:send_text(MAV_SEVERITY.NOTICE, string.format(
      "YawTune: started; target %.1f deg, max error %.1f deg, output %.0f-%.0f%%",
      math.deg(autotune_target_rad), max_error, min_output, max_output))
    gcs:send_text(MAV_SEVERITY.INFO, string.format("YawTune: relay switches at +/- %.1f deg", math.deg(hysteresis)))
  end
  local err = funcs:yawErrorRad(ahrs:get_yaw(), autotune_target_rad)
  local yaw_rate = ahrs:get_gyro():z()
  -- Output ramp: only before the first full relay cycle, so every measured
  -- cycle (the first two are discarded anyway) runs at one fixed amplitude
  if autotune.cycle == 0 then
    -- Breakaway: first output that really turns the rover (rate above 5 deg/s)
    if S.yaw_move_lvl == nil and math.abs(yaw_rate) >= S.YAW_MOVE_RATE and autotune.u ~= 0 then
      S.yaw_move_lvl = autotune.relay
    end
  end
  if autotune.cycle == 0 and autotune.relay < S.yaw_max_relay then
    if math.abs(yaw_rate) < S.yaw_ramp_rate then
      S.yaw_still_s = S.yaw_still_s + AUTOTUNE_PERIOD_MS / 1000.0
    else
      S.yaw_still_s = 0
    end
    if S.yaw_still_s >= S.YAW_RAMP_STILL_S then
      S.yaw_still_s = 0
      -- Last output at which the rover stayed still (under 2 deg/s for the whole second)
      if math.abs(yaw_rate) < YAW_RATE_THRESH_RAD_S and S.yaw_move_lvl == nil then
        S.yaw_still_lvl = autotune.relay
      end
      autotune.relay = math.min(S.yaw_max_relay, autotune.relay + S.YAW_RAMP_STEP)
      -- The relay holds its last output until the next switch; apply the new
      -- amplitude now, same direction
      if autotune.u ~= 0 then
        autotune.u = (autotune.u > 0) and autotune.relay or -autotune.relay
      end
      gcs:send_text(MAV_SEVERITY.INFO, string.format("YawTune: turn %.0f<%.0f deg/s; output %.0f%%",
        math.deg(math.abs(yaw_rate)), math.deg(S.yaw_ramp_rate), autotune.relay * 100))
    end
  end
  autotune.max_amp_rad = max_error_rad
  local s_out, status = autotune:step(err, AUTOTUNE_PERIOD_MS / 1000.0)
  yaw_tune_last_s_out = s_out
  applyControlAllocation(0, s_out)
  if autotune.cycle > yaw_tune_last_progress or autotune.t - yaw_tune_last_log_s >= 2 then
    yaw_tune_last_progress = autotune.cycle
    yaw_tune_last_log_s = autotune.t
    gcs:send_text(MAV_SEVERITY.INFO, string.format(
      "YawTune: t %.0fs cycle %d/%d err %.1fdeg rate %.1fdeg/s out %.2f",
      autotune.t, autotune.cycle, autotune.skip_cycles + autotune.cycles,
      math.deg(err), math.deg(yaw_rate), s_out))
  end
  if status == "done" then
    local r = autotune:result()
    local gain_multiplier = getYawGainMultiplier()
    local effective_p = r.P * gain_multiplier
    local effective_i = r.I * gain_multiplier
    param:set_and_save('SCR_USER2', r.P)
    param:set_and_save('SCR_USER3', r.I)
    S.saveYawBreakaway()
    steering_steady_pid:setGains(effective_p, effective_i, 0)
    steering_steady_pid:resetInternalState()
    gcs:send_text(MAV_SEVERITY.NOTICE, string.format("YawTune: Ku=%.2f Pu=%.2fs a=%.1fdeg", r.Ku, r.Pu, math.deg(r.amp)))
    gcs:send_text(MAV_SEVERITY.NOTICE, string.format(
      "YawTune: saved base P=%.3f I=%.3f; using P=%.3f I=%.3f (x%.2f)",
      r.P, r.I, effective_p, effective_i, gain_multiplier))
    if param:get(TUNE_SELECT_PARAM) == TUNE_MODE.YAW then
      param:set_and_save(TUNE_SELECT_PARAM, TUNE_MODE.NORMAL)
      gcs:send_text(MAV_SEVERITY.NOTICE, "YawTune: SCR_USER1 set to 0 (normal)")
    end
    autotune_done = true
    applyControlAllocation(0, 0)
    startYawVerificationSequence()
  elseif status == "fail" then
    gcs:send_text(MAV_SEVERITY.ERROR, "YawTune: failed - " .. autotune:result().msg)
    gcs:send_text(MAV_SEVERITY.WARNING, "YawTune: steering output stopped; selector remains available for retry")
    autotune_done = true
    applyControlAllocation(0, 0)
  end
  return true
end

-------------------------------------------------------------------------------
-------------------------------- MAIN LOOP ------------------------------------
-------------------------------------------------------------------------------
--[[
Main update function for the rover control allocation script
--]]
local function update()
  -- Safety check for vehicle type
  if not (VEHICLE_TYPE == 2) then
    gcs:send_text(MAV_SEVERITY.WARNING, string.format("Not ROVER, exiting LUA script."))
    return
  end
  -- Getting SCR_USER params to PID values
  -- local p, i, d = param:get('SCR_USER2') / 1000, param:get('SCR_USER3') / 1000, param:get('SCR_USER4') / 1000
  -- steering_steady_pid:setGains(p, i, d)

  -- Getting radio type
  radio_type = S.rc3_reversed:get() or 0

  -- Run not armed routine to guarantee trim values
  if not arming:is_armed() then
    if yaw_sequence_active then
      gcs:send_text(MAV_SEVERITY.WARNING, 'YawTune: heading sequence aborted (disarmed)')
    end
    yaw_sequence_active = false
    yaw_sequence_index = 0
    if autotune.status == "running" then
      gcs:send_text(MAV_SEVERITY.WARNING, "YawTune: aborted (disarmed)")
    end
    selector_armed_reported = false
    resetYawControlState()
    last_nav_idx = nil
    -- reset() allocates new tables; skip it when there is nothing to clear
    if autotune.status ~= "idle" then
      autotune:reset()
    end
    autotune_done = false
    yaw_tune_last_tick_ms = 0
    yaw_tune_last_s_out = 0
    circle_tune:reset("disarmed")
    -- The gain-set sync does many by-name lookups; keep it at 2 s while the
    -- loop itself polls fast, so the mixer reacts at once after arming.
    local now_ms = millis():toint()
    if now_ms - S.last_gain_sync_ms >= S.DISARMED_SYNC_MS then
      S.last_gain_sync_ms = now_ms
      syncGainSet()
      notArmed()
    end
    return update, S.DISARMED_PERIOD_MS
  end

  local vehicle_mode = vehicle:get_mode()
  if yaw_sequence_active and vehicle_mode ~= DRIVING_MODES.STEERING then
    yaw_sequence_active = false
    yaw_sequence_index = 0
    resetYawControlState()
    gcs:send_text(MAV_SEVERITY.WARNING, 'YawTune: heading sequence aborted (mode changed)')
  elseif yaw_target_rad ~= nil and vehicle_mode ~= DRIVING_MODES.STEERING then
    resetYawControlState()
  end
  local selector = S.tune_select:get()
  if not selector_armed_reported then
    if selector == TUNE_MODE.YAW then
      gcs:send_text(MAV_SEVERITY.INFO, string.format("YawTune: SCR_USER1=2; starts in HOLD (mode %d)", vehicle_mode))
    elseif selector == TUNE_MODE.PIXHAWK then
      gcs:send_text(MAV_SEVERITY.INFO, string.format("RTun: SCR_USER1=1; QuickTune enabled while armed (mode %d)", vehicle_mode))
    end
    selector_armed_reported = selector == TUNE_MODE.YAW or selector == TUNE_MODE.PIXHAWK
  end
  if selector ~= TUNE_MODE.YAW and (autotune.status ~= "idle" or autotune_done) then
    if autotune.status == "running" then
      gcs:send_text(MAV_SEVERITY.WARNING, "YawTune: aborted (SCR_USER1 changed)")
    end
    autotune:reset()
    autotune_done = false
    yaw_tune_last_tick_ms, yaw_tune_last_s_out = 0, 0
  elseif selector == TUNE_MODE.YAW then
    circle_tune:reset("selector changed")
    -- The relay owns the motors only in HOLD. Any other mode keeps normal
    -- control, and after a failure or abort the pilot gets the rover back.
    if autotune.status == "running" and vehicle_mode ~= DRIVING_MODES.HOLD then
      autotune:reset()
      autotune_done = true
      yaw_tune_last_tick_ms, yaw_tune_last_s_out = 0, 0
      applyControlAllocation(0, 0)
      gcs:send_text(MAV_SEVERITY.WARNING, "YawTune: aborted (left HOLD); set SCR_USER1 to 0 then 2 to retry")
    elseif autotune.status == "running" or (not autotune_done and vehicle_mode == DRIVING_MODES.HOLD) then
      runYawAutotune()
      if autotune.status == "fail" or autotune_done then applyControlAllocation(0, 0) end
      return update, modePeriod(AUTOTUNE_PERIOD_MS)
    elseif not autotune_done then
      local now_ms = millis():toint()
      if now_ms - S.yaw_hold_hint_ms >= 5000 then
        S.yaw_hold_hint_ms = now_ms
        gcs:send_text(MAV_SEVERITY.INFO, "YawTune: switch to HOLD to start")
      end
    end
  end
  if selector ~= TUNE_MODE.CIRCLE then
    circle_tune:reset("selector changed")
  elseif vehicle_mode == DRIVING_MODES.CIRCLE then
    local throttle = vehicle:get_control_output(THROTTLE_CONTROL_OUTPUT_CHANNEL)
    local native_steering = vehicle:get_control_output(CONTROL_OUTPUT_YAW)
    local steering, allow_output = circle_tune:update(throttle, native_steering, throttle_max_lua_param:get())
    if allow_output then
      -- circle_tune checks the throttle in ArduPilot's sign; the wheels get the motor sign
      local pwm_l, pwm_r = applyControlAllocation(S.apThrottle(throttle), steering or native_steering, 60)
      if steering then
        circle_tune:log_sample(throttle, steering, pwm_l, pwm_r)
      end
    end
    return update, CircleTune.PERIOD_MS
  else
    circle_tune:cancel("left Circle")
  end
  if vehicle_mode == DRIVING_MODES.MANUAL then
    applyPWMManualMode()
    return update, modePeriod(200)
  elseif vehicle_mode == DRIVING_MODES.STEERING then
    -- We use steering mode for yaw alignment
    applyPWMSteeringMode()
    return update, modePeriod(200)
  elseif vehicle_mode == DRIVING_MODES.AUTO then
    -- Same actuator rate as the Circle where QuickTune identified ATC_*:
    -- at 200 ms the PWM path adds ~0.2 s of delay and the tuned loops oscillate.
    applyPWMAutoMode()
    return update, CircleTune.PERIOD_MS
  elseif vehicle_mode == DRIVING_MODES.SIMPLE then
    applyPWMInternalControl()
    return update, CircleTune.PERIOD_MS
  elseif vehicle_mode == DRIVING_MODES.CIRCLE then
    -- Keep the same actuator update period used to identify the steering loop.
    applyPWMInternalControl(60)
    return update, CircleTune.PERIOD_MS
  elseif vehicle_mode == DRIVING_MODES.ACRO or vehicle_mode == DRIVING_MODES.RTL then
    applyPWMInternalControl()
    return update, CircleTune.PERIOD_MS
  elseif vehicle_mode == DRIVING_MODES.HOLD or vehicle_mode == DRIVING_MODES.GUIDED then
    -- Make the vehicle stop
    applyControlAllocation(0, 0)
    return update, modePeriod(200)
  end
  -- Unhandled mode: stop safely but keep the script alive
  applyControlAllocation(0, 0)
  return update, modePeriod(200)
end

gcs:send_text(MAV_SEVERITY.NOTICE, string.format(
  "Control allocation loaded; vehicle type=%.1f, SCR_USER1=%.1f, YAWT_MAX_ANG_ERR=%.1f deg, YAWT_MAX_OUT=%.0f%%",
  VEHICLE_TYPE or -1, param:get(TUNE_SELECT_PARAM) or -1, yaw_max_error_param:get(), yaw_max_out_param:get()))
return update, 3000 -- run immediately before starting to reschedule
-------------------------------------------------------------------------------
