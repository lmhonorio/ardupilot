--[[
Rover mixer: throttle / steering from the pilot or from ArduPilot's own
controllers, per driving mode, to the left / right wheels.

The output goes to the CAN driver (can_esc.lua) when it is installed and
active, otherwise to the PWM path (out_pwm.lua). Either output module may be
removed from APM/scripts/modules; the mixer then uses the one left.

Tunes it hands the motors to while they run:
  SCR_USER1 = 2  yaw_tune.lua (HOLD)
  SCR_USER1 = 3  circle-tune.lua (Circle)
QuickTune (SCR_USER1 = 1) only measures; ArduPilot's Circle drives.
--]]
if ... == nil then
  return
end

local R = require("rover_common")
local funcs = require("functions")
local CircleTune = require("circle-tune")
local GB = require("gain_banks")
local YA = require("yaw_align")
local YT = require("yaw_tune")
local MAV_SEVERITY = R.MAV_SEVERITY
local DRIVING_MODES = R.DRIVING_MODES
local MISSION_STATE = R.MISSION_STATE
local TUNE_MODE = R.TUNE_MODE
local PWM_TRIM_VALUE = R.PWM_TRIM_VALUE

local MX = {}

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
R.thro_max_lua = Parameter()
assert(R.thro_max_lua:init('THRO_MAX_LUA'), 'could not bind THRO_MAX_LUA parameter')
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
R.thro_min_lua = Parameter()
assert(R.thro_min_lua:init('THRO_MIN_LUA'), 'could not bind THRO_MIN_LUA parameter')
--[[
  // @Param: THRO_INV
  // @DisplayName: Invert ArduPilot throttle
  // @Description: 1 reverses the throttle that ArduPilot's own modes send (Auto, Circle, Acro, RTL, Simple, Circle tune) before the wheels. The manual stick path is never changed: it already takes forward as (trim - stick), the opposite of ArduPilot's throttle sign, so with 1 forward means forward in every mode. 0 keeps ArduPilot's sign.
  // @Values: 0:ArduPilot sign,1:Inverted (match the manual stick)
  // @User: Standard
--]]
assert(param:add_param(THROTTLE_PARAM_TABLE_KEY, 3, 'INV', 1), 'could not add THRO_INV parameter')
local thro_inv = Parameter()
assert(thro_inv:init('THRO_INV'), 'could not bind THRO_INV parameter')

-- Vehicle type control
local VEHICLE_TYPE = param:get('SCR_USER5')
-- TRIM values
local RC1_TRIM_VALUE = param:get('RC1_TRIM')
local RC3_TRIM_VALUE = param:get('RC3_TRIM')

local S = {
  -- Disarmed: poll at 200 ms so arming is picked up quickly
  DISARMED_PERIOD_MS = 200,
  DISARMED_SYNC_MS = 2000,
  last_gain_sync_ms = -2000,
  -- In CAN the mixer runs faster so the driver adds little delay
  CAN_MIXER_PERIOD_MS = 20,
  -- Read every cycle: bound objects instead of by-name lookups
  tune_select = Parameter(),
  rc3_reversed = Parameter(),
  radio_type = 0,
  selector_armed_reported = false,
  -- Manual stick smoothing (PWM path)
  last_manual_throttle = 0,
  throttle_accel_rate_thresh = 0.5,
  throttle_accel_rate = 0.5,
  last_manual_steering = 0,
  steering_accel_rate_thresh = 0.6,
  steering_accel_rate = 0.6,
}
assert(S.tune_select:init('SCR_USER1'), 'could not bind SCR_USER1 parameter')
assert(S.rc3_reversed:init('RC3_REVERSED'), 'could not bind RC3_REVERSED parameter')
-- Register a visible AP_Param float (MAVLink/Full Parameter List), not a Lua-only setting.
-- Key 20 is reserved for MIXER_; keep it stable so saved values survive reboot.
assert(param:add_table(20, 'MIXER_', 1), 'could not add MIXER_ parameter table')
--[[
  // @Param: MIXER_FREQ
  // @DisplayName: Lua mixer update frequency
  // @Description: Requested actuator mixer frequency in AUTO, CIRCLE (including CircleTune), ACRO, RTL and SIMPLE, for CAN and PWM. Changes apply on the next update. Does not change the yaw PI or wheel RPM control rates. Limited to 25-200 Hz; actual timing depends on scripting load.
  // @Units: Hz
  // @Range: 25 200
  // @Increment: 1
  // @User: Standard
--]]
assert(param:add_param(20, 1, 'FREQ', 50), 'could not add MIXER_FREQ parameter')
local mixer_freq = Parameter()
assert(mixer_freq:init('MIXER_FREQ'), 'could not bind MIXER_FREQ parameter')

local circle_tune = CircleTune:new()

local function mixerPeriod()
  local freq = mixer_freq:get()
  if freq == nil or freq ~= freq or freq <= 0 then
    freq = 50
  end
  -- CircleTune requires sample intervals below 50 ms. At 25 Hz the requested
  -- interval is 40 ms, also below its 60 ms PWM override timeout.
  freq = math.max(25, math.min(200, freq))
  return math.floor(1000 / freq + 0.5)
end

-- In CAN the mixer runs faster so the driver adds little delay
local function modePeriod(pwm_period_ms)
  if R.can_selected() then
    return math.min(pwm_period_ms, S.CAN_MIXER_PERIOD_MS)
  end
  return pwm_period_ms
end

-- Throttle from ArduPilot's controllers in the sign the wheels use. On this
-- rover forward is negative for the motors (manual forward logged -109 rpm on
-- all four wheels), and ArduPilot's positive throttle drove the rover backward
-- in Circle and Auto. Steering already matches and is not touched.
local function apThrottle(t)
  t = tonumber(t) or 0
  if thro_inv:get() >= 0.5 then
    return -t
  end
  return t
end

-- Stick deadzone for the CAN manual path: RCn_DZ when set, otherwise 20 us
local function stickDeadzone(name)
  local dz = param:get(name) or 0
  if dz > 0 then
    return math.min(dz, 200)
  end
  return 20
end
local RC1_DZ_US = stickDeadzone('RC1_DZ')
local RC3_DZ_US = stickDeadzone('RC3_DZ')

-- Normalised stick (-1..1 at +/-450 us) with the deadzone removed and the
-- rest of the travel rescaled, so full stick still gives full command
local function stickNorm(pwm, trim, dz_us)
  local delta = pwm - trim
  if math.abs(delta) <= dz_us then
    return 0
  end
  delta = (delta > 0) and (delta - dz_us) or (delta + dz_us)
  return delta / (450 - dz_us)
end

--[[
Control allocation: throttle t and steering s (-1..1) to the wheels.
CAN: skid-steer sum left = t + s, right = t - s, both sides scaled together
when the sum passes 1 so a turn while moving does not drop one stick. The
Circle relay uses this same mix, so the gains it identifies match the plant
seen in normal CAN driving.
PWM: the same mix as servo pulses (out_pwm.lua).
-- @return pwm_l, pwm_r - the PWM pulses for this command (logged by the Circle tune)
--]]
function MX.apply(t, s, timeout_ms)
  timeout_ms = timeout_ms or 300
  local pwm = R.pwm
  local pwm_l, pwm_r = PWM_TRIM_VALUE, PWM_TRIM_VALUE
  if pwm ~= nil then
    pwm_l, pwm_r = pwm.allocate(t, s)
  end
  if R.can_selected() then
    local left, right = t + s, t - s
    local peak = math.max(1, math.abs(left), math.abs(right))
    R.can.command(left / peak, right / peak)
    return pwm_l, pwm_r
  end
  if pwm == nil or not R.pwm_allowed() then
    return pwm_l, pwm_r
  end
  pwm.write(pwm_l, pwm_r, timeout_ms)
  return pwm_l, pwm_r
end
local apply = MX.apply

--[[
Control the actions while not armed
--]]
local function notArmed()
  if R.can_selected() then
    -- A trim pulse on MAIN 1..4 knocks the C620 off CAN; the CAN driver holds them at 0
    R.can.command(0, 0)
    S.last_manual_throttle = 0
    S.last_manual_steering = 0
    return
  end
  if R.pwm == nil or not R.pwm_allowed() then
    return
  end
  R.pwm.trim()
end

--[[
Perform vehicle control in Manual mode
--]]
local function manualMode()
  -- get_pwm may return nil; treat that as a centred stick, not a script error
  local rc3_pwm = rc:get_pwm(3) or RC3_TRIM_VALUE
  local rc1_pwm = rc:get_pwm(1) or RC1_TRIM_VALUE
  local raw_throttle = 0

  if R.can_selected() then
    -- CAN path is fully reactive: no Lua stick smoothing. The C620 PWM input
    -- has its own neutral band; the CAN speed loop does not, so a few us of
    -- stick jitter around trim became a small rpm command that made the
    -- wheels creep and shake. Apply the deadzone here.
    local throttle_sign = (S.radio_type == 1) and 1 or -1
    apply(stickNorm(rc3_pwm, RC3_TRIM_VALUE, RC3_DZ_US) * throttle_sign,
      stickNorm(rc1_pwm, RC1_TRIM_VALUE, RC1_DZ_US))
    return
  end

  -- Compares the diff from the last manual signals to the maximum rate we are accepting
  -- Make the actual command be a rate from the last to the required command if necessary
  local raw_steering = (rc1_pwm - RC1_TRIM_VALUE) / 450
  local steering = funcs:applyAbsSmoothing(raw_steering, S.last_manual_steering, S.steering_accel_rate_thresh,
    S.steering_accel_rate)
  S.last_manual_steering = steering
  if S.radio_type == 1 then
    raw_throttle = (rc3_pwm - RC3_TRIM_VALUE) / 450
  else
    raw_throttle = (RC3_TRIM_VALUE - rc3_pwm) / 450
  end
  local throttle = funcs:applyAbsSmoothing(raw_throttle, S.last_manual_throttle, S.throttle_accel_rate_thresh,
    S.throttle_accel_rate)
  S.last_manual_throttle = throttle

  apply(throttle, steering)
end

--[[
Perform vehicle control in Auto mode
--]]
local function autoMode()
  YA.auto_restart_check()

  -- Controls end of mission
  local mission_state = mission:state()
  if mission_state == MISSION_STATE.FINISHED then
    apply(0, 0)
    vehicle:set_mode(DRIVING_MODES.MANUAL)
    return
  end

  -- If we reached a waypoint, check if we need to align yaw from its altitude field
  if YA.auto_trigger() then
    return
  end

  -- Acquiring throttle and steering from internal control output
  -- The throttle sign is preserved so MAV_CMD_DO_SET_REVERSE can be handled natively by ArduRover.
  local throttle = apThrottle(vehicle:get_control_output(THROTTLE_CONTROL_OUTPUT_CHANNEL))
  local steering = tonumber(vehicle:get_control_output(CONTROL_OUTPUT_YAW)) or 0
  apply(throttle * MX.approach_scale(), steering)
end

--[[
Approach slowdown. ArduRover plans to pass a waypoint at speed toward the
next one; it does not know the mixer stops there to align the heading (Alt
not -1). Arriving at 0.5 m/s the rover slid 6-8 cm after the wheels stopped.
Over the last APPROACH_M before such a waypoint the throttle is scaled down
with the distance left, to APPROACH_MIN at the point. Pass-through waypoints
(Alt = -1) are left alone.
--]]
S.APPROACH_M = 0.5
S.APPROACH_MIN = 0.25
S.approach_idx = nil
S.approach_align = false
function MX.approach_scale()
  local idx = mission:get_current_nav_index()
  if idx == nil then
    return 1
  end
  -- The mission item is read once per waypoint, not every cycle
  if idx ~= S.approach_idx then
    S.approach_idx = idx
    local item = mission:get_item(idx)
    S.approach_align = item ~= nil and item:command() == 16 and item:z() ~= -1
  end
  if not S.approach_align then
    return 1
  end
  local dist = vehicle:get_wp_distance_m()
  if dist == nil or dist >= S.APPROACH_M then
    return 1
  end
  return S.APPROACH_MIN + (1 - S.APPROACH_MIN) * math.max(dist, 0) / S.APPROACH_M
end

--[[
Pass the ArduRover internal controllers' outputs straight to the allocation
(used by CIRCLE/ACRO/RTL, e.g. during Rover QuickTune)
--]]
local function internalControl(timeout_ms)
  local throttle = apThrottle(vehicle:get_control_output(THROTTLE_CONTROL_OUTPUT_CHANNEL))
  local steering = tonumber(vehicle:get_control_output(CONTROL_OUTPUT_YAW)) or 0
  apply(throttle, steering, timeout_ms)
end

--[[
One mixer cycle. Returns the delay to the next one in ms, or nil to stop.
--]]
function MX.step()
  -- Safety check for vehicle type
  if not (VEHICLE_TYPE == 2) then
    gcs:send_text(MAV_SEVERITY.WARNING, string.format("Not ROVER, exiting LUA script."))
    return nil
  end

  -- Getting radio type
  S.radio_type = S.rc3_reversed:get() or 0

  -- Run not armed routine to guarantee trim values
  if not arming:is_armed() then
    YA.on_disarm()
    YT.on_disarm()
    S.selector_armed_reported = false
    YA.on_disarm_reset()
    YT.on_disarm_reset()
    circle_tune:reset("disarmed")
    -- The gain-set sync does many by-name lookups; keep it at 2 s while the
    -- loop itself polls fast, so the mixer reacts at once after arming.
    local now_ms = millis():toint()
    if now_ms - S.last_gain_sync_ms >= S.DISARMED_SYNC_MS then
      S.last_gain_sync_ms = now_ms
      GB.sync()
      notArmed()
    end
    return S.DISARMED_PERIOD_MS
  end

  local vehicle_mode = vehicle:get_mode()
  YA.check_mode(vehicle_mode)
  local selector = S.tune_select:get()
  if not S.selector_armed_reported then
    if selector == TUNE_MODE.YAW then
      gcs:send_text(MAV_SEVERITY.INFO, string.format("YawTune: SCR_USER1=2; starts in HOLD (mode %d)", vehicle_mode))
    elseif selector == TUNE_MODE.PIXHAWK then
      gcs:send_text(MAV_SEVERITY.INFO, string.format("Tune: SCR_USER1=1, mede ao armar (modo %d)", vehicle_mode))
    end
    S.selector_armed_reported = selector == TUNE_MODE.YAW or selector == TUNE_MODE.PIXHAWK
  end
  if selector == TUNE_MODE.YAW then
    circle_tune:reset("selector changed")
  end
  if YT.handle(selector, vehicle_mode) then
    return modePeriod(YA.UPDATE_PERIOD_MS)
  end
  if selector ~= TUNE_MODE.CIRCLE then
    circle_tune:reset("selector changed")
  elseif vehicle_mode == DRIVING_MODES.CIRCLE then
    local throttle = vehicle:get_control_output(THROTTLE_CONTROL_OUTPUT_CHANNEL)
    local native_steering = vehicle:get_control_output(CONTROL_OUTPUT_YAW)
    local steering, allow_output = circle_tune:update(throttle, native_steering, R.thro_max_lua:get())
    if allow_output then
      -- circle_tune checks the throttle in ArduPilot's sign; the wheels get the motor sign
      local pwm_l, pwm_r = apply(apThrottle(throttle), steering or native_steering, 60)
      if steering then
        circle_tune:log_sample(throttle, steering, pwm_l, pwm_r)
      end
    end
    return mixerPeriod()
  else
    circle_tune:cancel("left Circle")
  end
  if vehicle_mode == DRIVING_MODES.MANUAL then
    manualMode()
    return modePeriod(200)
  elseif vehicle_mode == DRIVING_MODES.STEERING then
    -- We use steering mode for yaw alignment
    YA.steering_mode()
    return modePeriod(200)
  elseif vehicle_mode == DRIVING_MODES.AUTO then
    -- Use the same configurable actuator rate as Circle/QuickTune.
    autoMode()
    return mixerPeriod()
  elseif vehicle_mode == DRIVING_MODES.SIMPLE then
    internalControl()
    return mixerPeriod()
  elseif vehicle_mode == DRIVING_MODES.CIRCLE then
    -- Keep the same actuator update period used to identify the steering loop.
    internalControl(60)
    return mixerPeriod()
  elseif vehicle_mode == DRIVING_MODES.ACRO or vehicle_mode == DRIVING_MODES.RTL then
    internalControl()
    return mixerPeriod()
  elseif vehicle_mode == DRIVING_MODES.HOLD or vehicle_mode == DRIVING_MODES.GUIDED then
    -- Make the vehicle stop
    apply(0, 0)
    return modePeriod(200)
  end
  -- Unhandled mode: stop safely but keep the mixer alive
  apply(0, 0)
  return modePeriod(200)
end

-- After a mixer fault: hold the wheels stopped
function MX.stop_outputs()
  if R.can_selected() then
    R.can.command(0, 0)
  elseif R.pwm ~= nil and R.pwm_allowed() then
    R.pwm.trim()
  end
end

-- Scheduler: first cycle 3 s after load
MX.FIRST_DELAY_MS = 3000

gcs:send_text(MAV_SEVERITY.NOTICE, string.format(
  "Control allocation loaded; vehicle type=%.1f, SCR_USER1=%.1f, YAWT_MAX_ANG_ERR=%.1f deg, YAWT_MAX_OUT=%.0f%%",
  VEHICLE_TYPE or -1, param:get('SCR_USER1') or -1, param:get('YAWT_MAX_ANG_ERR'), param:get('YAWT_MAX_OUT')))

R.mixer = MX
return MX
