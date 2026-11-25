--[[
Yaw alignment: rotate in place to a heading with the steering PI.

Used by
  - AUTO: a NAV_WAYPOINT whose altitude (z) holds a heading in degrees stops
    the rover there and turns it in STEERING mode (z = -1 passes through);
  - the yaw tune's verification sequence (0, 90, 180, 270, 360 deg).
Gains: SCR_USER2 (P) and SCR_USER3 (I) times YAWT_GAIN_MULT.
--]]
if ... == nil then
  return
end

local R = require("rover_common")
local PID = require("pid")
local funcs = require("functions")
local MAV_SEVERITY = R.MAV_SEVERITY
local DRIVING_MODES = R.DRIVING_MODES

local YA = {}

YA.UPDATE_PERIOD_MS = 200
YA.UPDATE_DT = YA.UPDATE_PERIOD_MS / 1000.0
YA.YAW_THRESH_RAD = math.rad(2)
YA.YAW_RATE_THRESH_RAD_S = math.rad(2)
local YAW_SETTLE_STEPS = 3 -- require 600 ms continuously within angle and rate limits
local YAW_DEADBAND = 0.02
local YAW_DEADBAND_CAN = 0.005 -- 0.6 rpm at ESC_RPM 120: below the CAN stall-boost minimum
local YAW_ALIGN_TIMEOUT_MS = 15000
local YAW_ALIGN_LOG_INTERVAL_MS = 1000
local YAW_SEQUENCE_DEGREES = { 0, 90, 180, 270, 360 }
local YAW_SEQUENCE_PAUSE_MS = 5000
local YAW_SEQUENCE_MOVEMENT_RAD = math.rad(1)
YA.YAW_SEQUENCE_DEFAULT_TIMEOUT_S = 15
-- In CAN the mixer runs every 20 ms but the yaw PI and relay keep their 200 ms step
local YAW_TICK_TOLERANCE_MS = 10

-- State
local st = {
  last_yaw_align_log_ms = 0,
  sequence_active = false,
  sequence_index = 0,
  sequence_pause_start = nil,
  sequence_last_movement_ms = nil,
  sequence_movement_reference_rad = nil,
  sequence_timeout_ms = nil,
  target_rad = nil,
  align_steps = 0,
  settle_steps = 0,
  last_tick_ms = 0,
  last_s_out = 0,
  last_nav_idx = nil,
}
YA.st = st

-- Params: p_gain, i_gain, d_gain, i_max, pid_max
YA.pid = PID:new(3.5, 8, 0, 1, 0.95)

-- Yaw tune / alignment parameters. The tune's own slots (1, 2, 4, 5, 6) are
-- added by yaw_tune.lua; keep every slot where it was saved.
YA.PARAM_TABLE_KEY = 16
assert(param:add_table(YA.PARAM_TABLE_KEY, 'YAWT_', 7), 'could not add yaw autotune parameter table')
--[[
  // @Param: YAWT_GAIN_MULT
  // @DisplayName: Yaw autotune gain multiplier
  // @Description: Multiplier applied to tuned P and I gains when aligning yaw
  // @Range: 0.1 10000
  // @Increment: 0.1
  // @User: Standard
--]]
assert(param:add_param(YA.PARAM_TABLE_KEY, 3, 'GAIN_MULT', 1), 'could not add yaw autotune gain multiplier parameter')
local yaw_gain_multiplier_param = Parameter()
assert(yaw_gain_multiplier_param:init('YAWT_GAIN_MULT'), 'could not bind YAWT_GAIN_MULT parameter')

--[[
  // @Param: YAWT_TIMEOUT
  // @DisplayName: Yaw verification inactivity timeout
  // @Description: Time without at least one degree of heading change before stopping the post-tune yaw verification
  // @Units: s
  // @Range: 1 120
  // @User: Standard
--]]
assert(param:add_param(YA.PARAM_TABLE_KEY, 7, 'TIMEOUT', YA.YAW_SEQUENCE_DEFAULT_TIMEOUT_S),
  'could not add yaw verification timeout parameter')
local yaw_timeout_param = Parameter()
assert(yaw_timeout_param:init('YAWT_TIMEOUT'), 'could not bind YAWT_TIMEOUT parameter')

function YA.gain_multiplier()
  return math.max(0.1, math.min(10000, yaw_gain_multiplier_param:get()))
end

-- Load tuned gains if present (0 means "not tuned yet", keep defaults)
do
  local saved_p, saved_i = param:get('SCR_USER2') or 0, param:get('SCR_USER3') or 0
  if saved_p > 0 and saved_i > 0 then
    local gain_multiplier = YA.gain_multiplier()
    YA.pid:setGains(saved_p * gain_multiplier, saved_i * gain_multiplier, 0)
  end
end

local function apply(t, s, timeout_ms)
  return R.mixer.apply(t, s, timeout_ms)
end

--[[
Reset the yaw control state
--]]
function YA.reset()
  st.target_rad = nil
  st.sequence_pause_start = nil
  st.align_steps = 0
  st.settle_steps = 0
  YA.pid:resetInternalState()
  st.last_s_out = 0
  st.last_tick_ms = 0
end

-- True when the 200 ms yaw step is due; always true in PWM, where the yaw loops run at 200 ms
function YA.tick_due(last_tick_ms)
  if not R.can_selected() then
    return true, last_tick_ms
  end
  local now = millis():toint()
  if last_tick_ms == 0 or now - last_tick_ms >= YA.UPDATE_PERIOD_MS - YAW_TICK_TOLERANCE_MS then
    return true, now
  end
  return false, last_tick_ms
end

local function startVerificationHeading()
  YA.reset()
  st.target_rad = math.rad(YAW_SEQUENCE_DEGREES[st.sequence_index])
  st.sequence_movement_reference_rad = ahrs:get_yaw()
  st.sequence_last_movement_ms = millis()
  local timeout_s = yaw_timeout_param:get()
  if not (timeout_s >= 1 and timeout_s <= 120) then
    timeout_s = YA.YAW_SEQUENCE_DEFAULT_TIMEOUT_S
  end
  st.sequence_timeout_ms = timeout_s * 1000
  st.last_yaw_align_log_ms = 0
  gcs:send_text(MAV_SEVERITY.INFO, string.format(
    'YawTune: tentando orientar para %d° (etapa %d/%d)',
    YAW_SEQUENCE_DEGREES[st.sequence_index], st.sequence_index, #YAW_SEQUENCE_DEGREES))
end

-- After a yaw tune: turn to 0, 90, 180, 270 and 360 deg in STEERING
function YA.start_verification_sequence()
  st.sequence_active = true
  st.sequence_index = 1
  gcs:send_text(MAV_SEVERITY.NOTICE, 'YawTune: tentando orientar: 0°, 90°, 180°')
  gcs:send_text(MAV_SEVERITY.NOTICE, 'YawTune: depois 270° e 360°/0°')
  startVerificationHeading()
  if not vehicle:set_mode(DRIVING_MODES.STEERING) then
    st.sequence_active = false
    YA.reset()
    gcs:send_text(MAV_SEVERITY.ERROR, 'YawTune: nao foi possivel entrar em STEERING; sequencia cancelada')
    apply(0, 0)
  end
end

local function advanceVerificationSequence()
  local completed_heading = YAW_SEQUENCE_DEGREES[st.sequence_index]
  gcs:send_text(MAV_SEVERITY.NOTICE, string.format(
    'YawTune: orientacao para %d° concluida', completed_heading))
  if st.sequence_index >= #YAW_SEQUENCE_DEGREES then
    st.sequence_active = false
    st.sequence_index = 0
    YA.reset()
    apply(0, 0)
    vehicle:set_mode(DRIVING_MODES.HOLD)
    gcs:send_text(MAV_SEVERITY.NOTICE, 'YawTune: sequencia concluida; rover em HOLD')
    return
  end

  YA.reset()
  st.sequence_pause_start = millis()
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
local function triggerOnReachedWaypoint()
  local idx = mission:get_current_nav_index()
  if not idx then
    return false
  end

  -- Initialize the index tracker
  if st.last_nav_idx == nil then
    st.last_nav_idx = idx
    return false
  end

  -- When the nav index changes, the previous waypoint was reached
  if idx ~= st.last_nav_idx then
    local reached_idx = st.last_nav_idx
    st.last_nav_idx = idx
    local item = mission:get_item(reached_idx)
    if not item then
      YA.reset()
      return false
    end

    -- Only handle NAV_WAYPOINT (16) with valid param4 (yaw)
    if item:command() ~= 16 then
      YA.reset()
      return false
    end

    local yaw_target_deg, is_pass_through = decodeYawFromWaypointZ(item:z())

    -- If angle == -1, treat as pass-through waypoint: do NOT switch modes
    -- This value must be used in the waypoint before a DO_SET_REVERSE transition
    -- so the mission remains in AUTO while ArduRover changes the driving direction.
    if is_pass_through then
      -- clear any previous target just in case
      YA.reset()
      return false
    end

    if yaw_target_deg == nil then
      YA.reset()
      return false
    end

    st.target_rad = math.rad(yaw_target_deg)

    -- Reset PID state and start alignment
    YA.pid:resetInternalState()
    st.align_steps = 0
    st.settle_steps = 0
    st.last_s_out = 0
    st.last_tick_ms = 0
    -- Stop the vehicle and take over yaw using STEERING mode
    apply(0, 0)
    vehicle:set_mode(DRIVING_MODES.STEERING)
    return true
  end
  return false
end

-- AUTO: a mission restart / rewind (the nav index went backwards) clears the tracker
function YA.auto_restart_check()
  local idx = mission:get_current_nav_index()
  if idx and st.last_nav_idx and idx < st.last_nav_idx then
    YA.reset()
    st.last_nav_idx = nil
  end
end

-- AUTO: true when a reached waypoint started an alignment (the mixer stops here)
function YA.auto_trigger()
  return triggerOnReachedWaypoint()
end

--[[
STEERING mode: turn in place to the target heading
--]]
function YA.steering_mode()
  -- If the pilot or failsafe switched modes, stop pursuing yaw alignment
  if vehicle:get_mode() ~= DRIVING_MODES.STEERING then
    YA.pid:resetInternalState()
    st.settle_steps = 0
    return
  end

  if st.sequence_pause_start then
    if (millis() - st.sequence_pause_start):tofloat() < YAW_SEQUENCE_PAUSE_MS then
      apply(0, 0)
      return
    end
    st.sequence_index = st.sequence_index + 1
    startVerificationHeading()
  end
  if st.target_rad == nil then
    apply(0, 0)
    return
  end

  local tick_due
  tick_due, st.last_tick_ms = YA.tick_due(st.last_tick_ms)
  if not tick_due then
    apply(0, st.last_s_out)
    return
  end

  local current_yaw_rad = ahrs:get_yaw()
  local now = millis()
  if st.sequence_active then
    local movement = funcs:yawErrorRad(current_yaw_rad, st.sequence_movement_reference_rad)
    if math.abs(movement) >= YAW_SEQUENCE_MOVEMENT_RAD then
      st.sequence_movement_reference_rad = current_yaw_rad
      st.sequence_last_movement_ms = now
    elseif (now - st.sequence_last_movement_ms):tofloat() >= st.sequence_timeout_ms then
      local failed_heading = YAW_SEQUENCE_DEGREES[st.sequence_index]
      st.sequence_active = false
      st.sequence_index = 0
      YA.reset()
      apply(0, 0)
      vehicle:set_mode(DRIVING_MODES.HOLD)
      gcs:send_text(MAV_SEVERITY.ERROR, string.format(
        'YawTune: %.1fs without movement at %d deg; HOLD', st.sequence_timeout_ms / 1000, failed_heading))
      return
    end
  elseif st.align_steps >= math.floor(YAW_ALIGN_TIMEOUT_MS / YA.UPDATE_PERIOD_MS) then
    apply(0, 0)
    YA.reset()
    vehicle:set_mode(DRIVING_MODES.AUTO)
    return
  else
    st.align_steps = st.align_steps + 1
  end

  local err = funcs:yawErrorRad(current_yaw_rad, st.target_rad)
  local now_ms = now:toint()
  if now_ms - st.last_yaw_align_log_ms >= YAW_ALIGN_LOG_INTERVAL_MS then
    st.last_yaw_align_log_ms = now_ms
    gcs:send_text(MAV_SEVERITY.INFO, string.format(
      "YawAlign target=%.1f current=%.1f error=%.1f",
      math.deg(st.target_rad), math.deg(current_yaw_rad), math.deg(err)))
  end

  -- Require both heading and yaw rate to settle before stopping. This avoids
  -- switching to HOLD while the rover is still rotating through the target.
  local yaw_rate = ahrs:get_gyro():z()
  if math.abs(err) <= YA.YAW_THRESH_RAD and math.abs(yaw_rate) <= YA.YAW_RATE_THRESH_RAD_S then
    st.settle_steps = st.settle_steps + 1
  else
    st.settle_steps = 0
  end

  if st.settle_steps >= YAW_SETTLE_STEPS then
    apply(0, 0)
    YA.pid:resetInternalState()
    st.settle_steps = 0
    if st.sequence_active then
      advanceVerificationSequence()
    else
      -- Set HOLD mode so the vehicle stops for the external ROS photo routine
      YA.reset()
      vehicle:set_mode(DRIVING_MODES.HOLD)
    end
    return
  end

  -- Rotate in place with pid output.
  -- Clear the integrator once the error changes sign (target passed): on a
  -- long turn it wound up, and after the overshoot it cancelled the P term
  -- for seconds (Ti about 12 s), leaving a 10 degree error with a zero
  -- command (seen on CAN: target 0 rpm for 7 s at -11 to -9 degrees).
  if YA.pid.integrator * err < 0 then
    YA.pid.integrator = 0
  end
  local s_out = YA.pid:compute(err, YA.UPDATE_DT)
  -- PWM only: the ESC gets a raw duty, so a stalled rover needs a minimum kick.
  -- On CAN the command is a wheel speed target and the CAN driver breaks the
  -- wheels free itself (ESC_STALL), so no kick there.
  local can = R.can_selected()
  if not can and R.pwm ~= nil and math.abs(err) > YA.YAW_THRESH_RAD and
      math.abs(yaw_rate) <= YA.YAW_RATE_THRESH_RAD_S and s_out * err > 0 then
    s_out = R.pwm.min_command(s_out)
  end
  -- The deadband keeps a PWM ESC from chattering. On CAN the command is a wheel
  -- speed target that the driver can follow down to about 1 rpm, and 0.02
  -- (2.4 rpm) zeroed every command for heading errors under about 6 degrees.
  if math.abs(s_out) < (can and YAW_DEADBAND_CAN or YAW_DEADBAND) then
    s_out = 0
  end
  st.last_s_out = s_out
  apply(0, s_out)
end

-- Disarmed: drop the sequence and every target
function YA.on_disarm()
  if st.sequence_active then
    gcs:send_text(MAV_SEVERITY.WARNING, 'YawTune: heading sequence aborted (disarmed)')
  end
  st.sequence_active = false
  st.sequence_index = 0
end

function YA.on_disarm_reset()
  YA.reset()
  st.last_nav_idx = nil
end

-- Armed, every mixer cycle: a mode change ends the sequence or the alignment
function YA.check_mode(vehicle_mode)
  if st.sequence_active and vehicle_mode ~= DRIVING_MODES.STEERING then
    st.sequence_active = false
    st.sequence_index = 0
    YA.reset()
    gcs:send_text(MAV_SEVERITY.WARNING, 'YawTune: heading sequence aborted (mode changed)')
  elseif st.target_rad ~= nil and vehicle_mode ~= DRIVING_MODES.STEERING then
    YA.reset()
  end
end

R.yaw_align = YA
return YA
