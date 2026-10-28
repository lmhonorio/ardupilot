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
-------------------------------------------------------------------------------
------------------------- GLOBAL SCOPE DEFINITIONS ----------------------------
-------------------------------------------------------------------------------
-- Channel control variables
local THROTTLE_CONTROL_OUTPUT_CHANNEL = 3
local CONTROL_OUTPUT_YAW = 4
local MAX_CHANNEL_OUTPUT = 1950
local MIN_CHANNEL_OUTPUT = 1050
local PWM_RANGE = 450
-- Vehicle type control
local VEHICLE_TYPE = param:get('SCR_USER5')
-- TRIM values
local PWM_TRIM_VALUE = tonumber(param:get('SERVO1_TRIM')) or 0
local RC1_TRIM_VALUE = param:get('RC1_TRIM')
local RC3_TRIM_VALUE = param:get('RC3_TRIM')
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
local YAW_DEADBAND = 0.02
local YAW_ALIGN_TIMEOUT_MS = 15000
-- Params: p_gain, i_gain, d_gain, i_max, pid_max
local steering_steady_pid = PID:new(3.5, 8, 0, 1, 0.95)
local yaw_target_rad = nil
local yaw_align_steps = 0

local last_nav_idx = nil

local radio_type = 0

-- Tuning selector (SCR_USER6):
--   0 = normal operation
--   1 = Pixhawk PID tune (Rover QuickTune)     - starts when entering CIRCLE
--   2 = yaw PI autotune (relay, AHRS heading)  - starts when entering HOLD
--   3 = wheel-speed PI autotune (relay, CAN)   - starts when entering HOLD
-- After a successful tune the selector returns to 0 automatically.
-- Yaw can also be triggered in HOLD by an RC switch with RCx_OPTION = 301.
-- Yaw results: SCR_USER2 (P), SCR_USER3 (I). Wheel results: C620_P / C620_I.
local TUNE_SELECT_PARAM = 'SCR_USER6'
local TUNE_MODE = { NORMAL = 0, PIXHAWK = 1, YAW = 2, WHEEL = 3 }
local AUTOTUNE_RC_FUNC = 301
local AUTOTUNE_PERIOD_MS = 50
local WHEEL_TUNE_PERIOD_MS = 20
local autotune = Autotune:new({ relay = 0.45 })
local autotune_target_rad = nil
local autotune_done = false
local wheel_autotune = nil
local wheel_autotune_done = false

-- Load tuned yaw gains if present (0 means "not tuned yet", keep defaults)
local saved_p, saved_i = param:get('SCR_USER2') or 0, param:get('SCR_USER3') or 0
if saved_p > 0 and saved_i > 0 then
  steering_steady_pid:setGains(saved_p, saved_i, 0)
end

-- Severity for logging in GCS
MAV_SEVERITY = { EMERGENCY = 0, ALERT = 1, CRITICAL = 2, ERROR = 3, WARNING = 4, NOTICE = 5, INFO = 6, DEBUG = 7 }
-- Rover driving modes
DRIVING_MODES = { MANUAL = 0, ACRO = 1, STEERING = 3, HOLD = 4, CIRCLE = 9, AUTO = 10, RTL = 11, GUIDED = 15 }
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
-- CAN selected: hand the same left/right command to c620_can.lua and do not
-- pulse the servo pins. PWM selected: drive the pins and leave the CAN bus quiet.
local function can_selected()
  return C620 ~= nil and C620.output_is_can()
end

local function norm_from_pwm(pwm)
  local value = (pwm - PWM_TRIM_VALUE) / PWM_RANGE
  if value > 1 then
    return 1
  elseif value < -1 then
    return -1
  end
  return value
end

local function applyControlAllocation(t, s)
  -- We assign the PWM values to the motors, which are opposite in sign for each diagonal pair
  -- MOTOR SCHEMATIC IN ROVER FRAME
  -- 1 - 0     ^
  --   |       | Rover forward direction
  -- 2 - 3
  -- Setting the PWM outputs based on the control allocation directions
  -- left for motors in the left side (1 and 2), right for the ones on the right side (0 and 3)
  local pwm_aloc_l, pwm_aloc_r = funcs:allocateRightAndLeftPwmShare(t, s, PWM_RANGE)
  local pwm_l = funcs:mapMaxMin(PWM_TRIM_VALUE + pwm_aloc_l, MIN_CHANNEL_OUTPUT, MAX_CHANNEL_OUTPUT)
  local pwm_r = funcs:mapMaxMin(PWM_TRIM_VALUE - pwm_aloc_r, MIN_CHANNEL_OUTPUT, MAX_CHANNEL_OUTPUT)
  if can_selected() then
    C620.set_norms({
      norm_from_pwm(pwm_r),
      norm_from_pwm(pwm_l),
      norm_from_pwm(pwm_l),
      norm_from_pwm(pwm_r),
      0, 0, 0, 0
    })
    return
  end
  if C620 ~= nil then
    C620.clear_script_norms()
  end
  SRV_Channels:set_output_pwm_chan_timeout(0, pwm_r, 300)
  SRV_Channels:set_output_pwm_chan_timeout(1, pwm_l, 300)
  SRV_Channels:set_output_pwm_chan_timeout(2, pwm_l, 300)
  SRV_Channels:set_output_pwm_chan_timeout(3, pwm_r, 300)
end

local function stopMotors()
  applyControlAllocation(0, 0)
  if C620 ~= nil then
    C620.clear_current_override()
  end
end

-------------------------------------------------------------------------------
--------------------- YAW CONTROL VIA MAV_CMD_NAV_WAYPOINT --------------------
-------------------------------------------------------------------------------
--[[
Reset the yaw control state
--]]
local function resetYawControlState()
  yaw_target_rad = nil
  yaw_align_steps = 0
  steering_steady_pid:resetInternalState()
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
  -- Trim pulses on MAIN 1..4 make the C620 drop CAN. With CAN on, send no pulse.
  local pwm = PWM_TRIM_VALUE
  if can_selected() then
    pwm = 0
  end
  SRV_Channels:set_output_pwm_chan_timeout(0, pwm, 3000)
  SRV_Channels:set_output_pwm_chan_timeout(1, pwm, 3000)
  SRV_Channels:set_output_pwm_chan_timeout(2, pwm, 3000)
  SRV_Channels:set_output_pwm_chan_timeout(3, pwm, 3000)
end

--[[
Perform vehicle control in Manual mode
--]]
local function applyPWMManualMode()
  local rc3_pwm = rc:get_pwm(3)
  local rc1_pwm = rc:get_pwm(1)
  local raw_throttle = 0

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
    return
  end

  -- Timeout safety
  yaw_align_steps = yaw_align_steps + 1
  local yaw_align_max_steps = math.floor(YAW_ALIGN_TIMEOUT_MS / UPDATE_PERIOD_MS)
  if yaw_align_steps > yaw_align_max_steps then
    applyControlAllocation(0, 0)
    steering_steady_pid:resetInternalState()
    vehicle:set_mode(DRIVING_MODES.AUTO)
    return
  end

  -- Current yaw from AHRS (rad) and error to target
  local err = funcs:yawErrorRad(ahrs:get_yaw(), yaw_target_rad)

  -- Check if we reached the target yaw
  if math.abs(err) <= YAW_THRESH_RAD then
    applyControlAllocation(0, 0)
    steering_steady_pid:resetInternalState()
    -- Set HOLD mode so the vehicle stops for the external ROS photo routine
    vehicle:set_mode(DRIVING_MODES.HOLD)
    return
  end

  -- Rotate in place with pid output
  local s_out = steering_steady_pid:compute(err, UPDATE_DT)
  if math.abs(s_out) < YAW_DEADBAND then
    s_out = 0
  end
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
  local throttle = tonumber(vehicle:get_control_output(THROTTLE_CONTROL_OUTPUT_CHANNEL)) or 0
  local steering = tonumber(vehicle:get_control_output(CONTROL_OUTPUT_YAW)) or 0
  applyControlAllocation(throttle, steering)
end

--[[
Pass the ArduRover internal controllers' outputs straight to the allocation
(used by CIRCLE/ACRO/RTL, e.g. during Rover QuickTune)
--]]
local function applyPWMInternalControl()
  local throttle = tonumber(vehicle:get_control_output(THROTTLE_CONTROL_OUTPUT_CHANNEL)) or 0
  local steering = tonumber(vehicle:get_control_output(CONTROL_OUTPUT_YAW)) or 0
  applyControlAllocation(throttle, steering)
end

local function resetYawAutotune(reason)
  if autotune.status == "running" and reason then
    gcs:send_text(MAV_SEVERITY.WARNING, reason)
  end
  autotune:reset()
  autotune_done = false
end

local function resetWheelAutotune(reason)
  if wheel_autotune and wheel_autotune.status == "running" and reason then
    gcs:send_text(MAV_SEVERITY.WARNING, reason)
  end
  if C620 ~= nil then
    C620.clear_current_override()
  end
  if wheel_autotune then
    wheel_autotune:reset()
  end
  wheel_autotune = nil
  wheel_autotune_done = false
end

--[[
Yaw PI autotune in HOLD mode.
Switch high starts the relay test around the current yaw; switch low aborts/rearms.
-- @return bool - true while the autotune owns the outputs
--]]
local function runYawAutotune()
  local sw = rc:get_aux_cached(AUTOTUNE_RC_FUNC)
  if param:get(TUNE_SELECT_PARAM) == TUNE_MODE.YAW then
    sw = 2
  end
  if sw ~= 2 then
    if autotune.status == "running" then
      gcs:send_text(MAV_SEVERITY.WARNING, "YawTune: aborted by switch")
    end
    autotune:reset()
    autotune_done = false
    return false
  end
  if autotune_done then
    return false
  end

  if autotune.status == "idle" then
    autotune_target_rad = ahrs:get_yaw()
    autotune:start()
    gcs:send_text(MAV_SEVERITY.NOTICE, "YawTune: started")
  end

  local err = funcs:yawErrorRad(ahrs:get_yaw(), autotune_target_rad)
  local s_out, status = autotune:step(err, AUTOTUNE_PERIOD_MS / 1000.0)
  applyControlAllocation(0, s_out)

  if status == "done" then
    local r = autotune:result()
    param:set_and_save('SCR_USER2', r.P)
    param:set_and_save('SCR_USER3', r.I)
    steering_steady_pid:setGains(r.P, r.I, 0)
    steering_steady_pid:resetInternalState()
    gcs:send_text(MAV_SEVERITY.NOTICE, string.format("YawTune: Ku=%.2f Pu=%.2fs a=%.1fdeg", r.Ku, r.Pu, math.deg(r.amp)))
    gcs:send_text(MAV_SEVERITY.NOTICE, string.format("YawTune: saved P=%.3f I=%.3f", r.P, r.I))
    if param:get(TUNE_SELECT_PARAM) == TUNE_MODE.YAW then
      param:set_and_save(TUNE_SELECT_PARAM, TUNE_MODE.NORMAL)
      gcs:send_text(MAV_SEVERITY.NOTICE, "YawTune: SCR_USER6 set to 0 (normal)")
    end
    autotune_done = true
    stopMotors()
  elseif status == "fail" then
    gcs:send_text(MAV_SEVERITY.ERROR, "YawTune: failed - " .. autotune:result().msg)
    autotune_done = true
    stopMotors()
  end
  return true
end

local function make_wheel_autotune()
  local max_cur = param:get('C620_CUR') or 10
  local max_rpm = param:get('C620_RPM') or 400
  local relay = math.min(math.max(max_cur * 0.25, 0.5), 5)
  local hyst = math.max(max_rpm * 0.03, 8)
  return Autotune:new({
    relay = relay,
    hyst = hyst,
    skip_cycles = 2,
    cycles = 4,
    max_amp = math.max(max_rpm * 0.6, 80),
    timeout_s = 30,
    min_period_s = 0.08,
    max_period_s = 8,
    max_spread = 0.35,
  })
end

--[[
Inner-loop wheel-speed autotune in HOLD.
Applies a current relay on all four C620s and uses CAN encoder RPM as feedback.
Writes C620_P / C620_I (Tyreus-Luyben) and leaves C620_D = 0.
-- @return bool - true while the autotune owns the outputs
--]]
local function runWheelAutotune()
  if param:get(TUNE_SELECT_PARAM) ~= TUNE_MODE.WHEEL then
    return false
  end
  if wheel_autotune_done then
    return false
  end

  if C620 == nil or not C620.output_is_can() then
    gcs:send_text(MAV_SEVERITY.ERROR, "WheelTune: CAN is off (set C620_CAN=1)")
    wheel_autotune_done = true
    return true
  end
  if not C620.drive_ok() then
    gcs:send_text(MAV_SEVERITY.ERROR, "WheelTune: missing ESC feedback")
    wheel_autotune_done = true
    return true
  end

  if wheel_autotune == nil or wheel_autotune.status == "idle" then
    wheel_autotune = make_wheel_autotune()
    wheel_autotune:start()
    gcs:send_text(MAV_SEVERITY.NOTICE, string.format("WheelTune: started relay=%.2fA", wheel_autotune.relay))
  end

  local rpm = C620.mean_drive_rpm()
  if rpm == nil then
    gcs:send_text(MAV_SEVERITY.ERROR, "WheelTune: lost RPM")
    resetWheelAutotune(nil)
    wheel_autotune_done = true
    stopMotors()
    return true
  end

  -- Target 0 rpm: the rover rocks forward/back around rest. Error = target - measured.
  local u, status = wheel_autotune:step(-rpm, WHEEL_TUNE_PERIOD_MS / 1000.0)
  C620.set_current_amps({ u, u, u, u, 0, 0, 0, 0 })

  if status == "done" then
    local r = wheel_autotune:result()
    param:set_and_save('C620_P', r.P)
    param:set_and_save('C620_I', r.I)
    param:set_and_save('C620_D', 0)
    param:set_and_save('C620_FF', 0)
    param:set_and_save('C620_MODE', 0)
    gcs:send_text(MAV_SEVERITY.NOTICE, string.format("WheelTune: Ku=%.4f Pu=%.2fs a=%.1frpm", r.Ku, r.Pu, r.amp))
    gcs:send_text(MAV_SEVERITY.NOTICE, string.format("WheelTune: saved C620_P=%.4f C620_I=%.4f", r.P, r.I))
    param:set_and_save(TUNE_SELECT_PARAM, TUNE_MODE.NORMAL)
    gcs:send_text(MAV_SEVERITY.NOTICE, "WheelTune: SCR_USER6 set to 0 (normal)")
    wheel_autotune_done = true
    stopMotors()
  elseif status == "fail" then
    gcs:send_text(MAV_SEVERITY.ERROR, "WheelTune: failed - " .. wheel_autotune:result().msg)
    wheel_autotune_done = true
    stopMotors()
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

  -- Getting radio type
  radio_type = param:get('RC3_REVERSED') or 0

  -- Run not armed routine to guarantee trim values
  if not arming:is_armed() then
    resetYawControlState()
    last_nav_idx = nil
    resetYawAutotune(nil)
    resetWheelAutotune(nil)
    notArmed()
    return update, 2000
  end

  -- Run control output allocation based on the current driving mode
  local vehicle_mode = vehicle:get_mode()

  -- Leaving HOLD aborts/rearms the HOLD-based autotunes
  if vehicle_mode ~= DRIVING_MODES.HOLD then
    if autotune.status ~= "idle" or autotune_done then
      resetYawAutotune(autotune.status == "running" and "YawTune: aborted (left HOLD)" or nil)
    end
    if wheel_autotune ~= nil or wheel_autotune_done then
      resetWheelAutotune(wheel_autotune and wheel_autotune.status == "running" and "WheelTune: aborted (left HOLD)" or nil)
    end
  end

  if vehicle_mode == DRIVING_MODES.MANUAL then
    applyPWMManualMode()
    return update, 200
  elseif vehicle_mode == DRIVING_MODES.STEERING then
    -- We use steering mode for yaw alignment
    applyPWMSteeringMode()
    return update, 200
  elseif vehicle_mode == DRIVING_MODES.AUTO then
    applyPWMAutoMode()
    return update, 200
  elseif vehicle_mode == DRIVING_MODES.HOLD and runWheelAutotune() then
    return update, WHEEL_TUNE_PERIOD_MS
  elseif vehicle_mode == DRIVING_MODES.HOLD and runYawAutotune() then
    return update, AUTOTUNE_PERIOD_MS
  elseif vehicle_mode == DRIVING_MODES.CIRCLE or vehicle_mode == DRIVING_MODES.ACRO
      or vehicle_mode == DRIVING_MODES.RTL then
    applyPWMInternalControl()
    return update, 200
  elseif vehicle_mode == DRIVING_MODES.HOLD or vehicle_mode == DRIVING_MODES.GUIDED then
    -- Make the vehicle stop
    applyControlAllocation(0, 0)
    return update, 200
  end
  -- Unhandled mode: stop safely but keep the script alive
  applyControlAllocation(0, 0)
  return update, 200
end

return update, 3000 -- run immediately before starting to reschedule
-------------------------------------------------------------------------------
