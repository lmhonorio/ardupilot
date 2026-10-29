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
package.path = package.path .. ';./scripts/modules/?.lua'
local PID = require("pid")
local funcs = require("functions")
-- local mavlink_msgs = require("MAVLink/mavlink_msgs")
-- inicializa interface MAVLink apenas para envio (0 msgs rx, buffer 10)
-- mavlink:init(0, 10)
-------------------------------------------------------------------------------
------------------------- GLOBAL SCOPE DEFINITIONS ----------------------------
-------------------------------------------------------------------------------
-- Channel control variables
local THROTTLE_CONTROL_OUTPUT_CHANNEL = 3
local CONTROL_OUTPUT_YAW = 4
-- Vehicle type control
local vehicle_type_warning = false
-- Yaw alignment logic
local UPDATE_PERIOD_MS = 20
local UPDATE_DT = UPDATE_PERIOD_MS / 1000.0
local OUTPUT_TIMEOUT_MS = 100
local motor_outputs = {}
local outputs_overridden = false
local outputs_loaded = false
local YAW_THRESH_RAD = math.rad(10)
local YAW_DEADBAND = 0.02
local YAW_ALIGN_TIMEOUT_MS = 15000
local YAW_ALIGN_MAX_STEPS = math.floor(YAW_ALIGN_TIMEOUT_MS / UPDATE_PERIOD_MS)
-- Params: p_gain, i_gain, d_gain, i_max, i_min, pid_max, pid_min
local yaw_pid = PID:new(0.5, 0.25, 0.2, 5, -5, 0.99, -0.99)
local yaw_target_deg = nil
local yaw_target_rad = nil
local yaw_align_steps = 0

local last_nav_idx = nil

-- Enable the custom allocator explicitly; 0 leaves all modes native.
local act_obs_avoid_param = Parameter()
-- system-start.lua creates this parameter after scripts have been loaded.
local act_obs_avoid_bound = false
local waypoint_yaw_param = Parameter()
local waypoint_yaw_bound = false

-- ROI e controle de gimbal/mount
local roi_lat, roi_lon, roi_alt = nil, nil, nil
local last_nav_idx_for_roi = nil
local last_mount_update_ms = nil

-- Severity for logging in GCS
MAV_SEVERITY = { EMERGENCY = 0, ALERT = 1, CRITICAL = 2, ERROR = 3, WARNING = 4, NOTICE = 5, INFO = 6, DEBUG = 7 }
-- Rover driving modes
DRIVING_MODES = { MANUAL = 0, STEERING = 3, HOLD = 4, AUTO = 10, GUIDED = 15 }
-- Mission states dictionary
MISSION_STATE = { IDLE = 0, RUNNING = 1, FINISHED = 2 }

-- Controle de impressão de param4
local last_logged_wp_index = -1
local last_inverted_check_idx = -1

-------------------------------------------------------------------------------
---------------------------- ROI & MOUNT CONTROL ------------------------------
-------------------------------------------------------------------------------

local function deg2rad(x)
  return x * math.pi / 180.0
end

local function rad2deg(x)
  return x * 180.0 / math.pi
end

local function distance_and_bearing(lat1, lon1, lat2, lon2)
  local R = 6371000.0
  local phi1 = deg2rad(lat1)
  local phi2 = deg2rad(lat2)
  local dphi = deg2rad(lat2 - lat1)
  local dlambda = deg2rad(lon2 - lon1)

  local a = math.sin(dphi / 2.0) ^ 2 +
      math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2.0) ^ 2
  local c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

  local d = R * c

  local y = math.sin(dlambda) * math.cos(phi2)
  local x = math.cos(phi1) * math.sin(phi2) -
      math.sin(phi1) * math.cos(phi2) * math.cos(dlambda)
  local brg = (rad2deg(math.atan2(y, x)) + 360.0) % 360.0

  return d, brg
end

-- Atualiza ROI ativo a partir da missão (MAV_CMD_DO_SET_ROI legacy = 201)
local function update_active_roi_from_mission()
  local idx = mission:get_current_nav_index()
  if not idx then
    return
  end

  if last_nav_idx_for_roi == idx then
    return
  end
  last_nav_idx_for_roi = idx

  local item = mission:get_item(idx)
  if not item then
    return
  end

  if item:command() == 201 then
    roi_lat = item:x() * 1.0e-7
    roi_lon = item:y() * 1.0e-7
    roi_alt = item:z()
    gcs:send_text(
      MAV_SEVERITY.INFO,
      string.format("ROI atualizado da missão: lat=%.7f lon=%.7f alt=%.1f", roi_lat, roi_lon, roi_alt)
    )
  end
end

-- Envia DO_MOUNT_CONTROL para apontar o gimbal para o ROI atual
local function send_mount_control_to_roi()
  if roi_lat == nil or roi_lon == nil or roi_alt == nil then
    return
  end

  local loc = ahrs:get_location()
  if not loc then
    return
  end

  local veh_lat = loc:lat() * 1.0e-7
  local veh_lon = loc:lng() * 1.0e-7
  local veh_alt = loc:alt() * 0.01

  local dist_m, bearing = distance_and_bearing(veh_lat, veh_lon, roi_lat, roi_lon)
  if dist_m <= 0 then
    return
  end

  local veh_yaw_deg = rad2deg(ahrs:get_yaw())
  local yaw_body_deg = (bearing - veh_yaw_deg + 360.0) % 360.0

  local dz = roi_alt - veh_alt
  local pitch_deg = rad2deg(math.atan2(dz, dist_m))
end

-------------------------------------------------------------------------------
---------------------------- DEBUGGING LOGGING --------------------------------
-------------------------------------------------------------------------------
local function logCurrentWpParam4()
  local idx = mission:get_current_nav_index()
  if not idx then
    return
  end
  if idx == last_logged_wp_index then
    return
  end
  local item = mission:get_item(idx)
  if not item then
    return
  end

  local cmd = item:command()
  -- 16 = MAV_CMD_NAV_WAYPOINT
  if cmd == 16 then
    local p4 = item:param4() or 0
    gcs:send_text(
      MAV_SEVERITY.INFO,
      string.format("WP %d (NAV_WAYPOINT) param4 = %.3f", idx, p4)
    )
    last_logged_wp_index = idx
  end
end

-------------------------------------------------------------------------------
------------------------- LOW LEVEL ACTION FUNCTIONS --------------------------
-------------------------------------------------------------------------------
--[[
Control allocation
This function is responsible for the PWM signal allocation of the vehicle.
It receives the throttle (t) and steering (s) values and calculates the PWM values for the motors.
The function also takes into account the trim values for the PWM outputs.
-- @param t number - Signed throttle command from -1.0 to 1.0
-- @param s number - Steering command from -1.0 to 1.0
--]]
local function releaseControlAllocation()
  if outputs_overridden then
    for _, output in ipairs(motor_outputs) do
      -- timeout=0 hands the channel back to the native motor driver.
      SRV_Channels:set_output_pwm_chan_timeout(output.channel, output.trim, 0)
    end
    outputs_overridden = false
  end
end

local function loadMotorOutputs()
  releaseControlAllocation()
  outputs_loaded = false
  local pwm_type = param:get('MOT_PWM_TYPE')
  if pwm_type ~= nil and pwm_type ~= 0 then
    gcs:send_text(MAV_SEVERITY.ERROR, 'Allocation: requires normal bidirectional PWM (MOT_PWM_TYPE=0)')
    return false
  end
  if (param:get('WRC_ENABLE') or 0) ~= 0 then
    gcs:send_text(MAV_SEVERITY.ERROR, 'Allocation: wheel-rate controller requires native outputs')
    return false
  end
  local configured = {}
  local have_left, have_right = false, false
  for servo = 1, 32 do
    local prefix = string.format('SERVO%d_', servo)
    local fn = param:get(prefix .. 'FUNCTION')
    if fn == 73 or fn == 74 then
      local output = {
        channel = servo - 1, left = fn == 73,
        minimum = param:get(prefix .. 'MIN'), trim = param:get(prefix .. 'TRIM'),
        maximum = param:get(prefix .. 'MAX'), reversed = param:get(prefix .. 'REVERSED') == 1
      }
      if not funcs:normalizedToPwm(0, output.minimum, output.trim, output.maximum, false) then
        gcs:send_text(MAV_SEVERITY.ERROR, 'Allocation: invalid limits for ' .. prefix)
        return false
      end
      configured[#configured + 1] = output
      have_left = have_left or output.left
      have_right = have_right or not output.left
    end
  end
  motor_outputs = configured
  outputs_loaded = have_left and have_right
  return outputs_loaded
end

local function applyControlAllocation(t, s)
  local left, right = funcs:allocateRightAndLeft(t, s,
    param:get('MOT_STR_THR_MIX'), param:get('MOT_THST_ASYM'))
  local minimum, expo = param:get('MOT_THR_MIN'), param:get('MOT_THST_EXPO')
  left = funcs:scaleMotorOutput(left, minimum, expo)
  right = funcs:scaleMotorOutput(right, minimum, expo)
  for _, output in ipairs(motor_outputs) do
    local command = output.left and left or right
    local pwm = funcs:normalizedToPwm(command, output.minimum, output.trim, output.maximum, output.reversed)
    SRV_Channels:set_output_pwm_chan_timeout(output.channel, pwm, OUTPUT_TIMEOUT_MS)
  end
  outputs_overridden = true
end

local function applyAutopilotControl()
  -- The autopilot has already applied RCMAP, RC reversal, trim and deadzone.
  -- Keep signed commands, including stop, reverse and active braking.
  applyControlAllocation(vehicle:get_control_output(THROTTLE_CONTROL_OUTPUT_CHANNEL),
    vehicle:get_control_output(CONTROL_OUTPUT_YAW))
end

-------------------------------------------------------------------------------
--------------------- YAW CONTROL VIA MAV_CMD_NAV_WAYPOINT --------------------
-------------------------------------------------------------------------------
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
      return false
    end

    -- Only handle NAV_WAYPOINT (16) with valid param4 (yaw)
    if item:command() ~= 16 then
      return false
    end
    local p4 = item:param4()
    if not funcs:isFinite(p4) then
      return false
    end

    -- param4 is yaw angle in degrees
    yaw_target_deg = funcs:mapTo360(p4)
    yaw_target_rad = math.rad(yaw_target_deg)
    -- Reset PID state and start alignment
    yaw_pid:resetInternalState()
    yaw_align_steps = 0
    -- Stop the vehicle and take over yaw using STEERING mode
    applyControlAllocation(0, 0)
    vehicle:set_mode(DRIVING_MODES.STEERING)

    gcs:send_text(
      MAV_SEVERITY.INFO,
      string.format("Yaw align start (WP yaw=%.1f deg)", yaw_target_deg)
    )
    return true
  end
  return false
end

local function resetMissionYawState()
  yaw_target_deg = nil
  yaw_target_rad = nil
  yaw_align_steps = 0
  last_nav_idx = nil
  last_logged_wp_index = -1
  yaw_pid:resetInternalState()
end

-------------------------------------------------------------------------------
--------------------- INVERTED FLIGHT COMMAND CHECK ---------------------------
-------------------------------------------------------------------------------
--[[
Check if the current mission command is MAV_CMD_DO_INVERTED_FLIGHT (210)
and print a message to the GCS.
--]]
local function checkForInvertedFlightCommand()
  local idx = mission:get_current_nav_index()
  if not idx then
    return
  end

  -- Avoid repeating the check for the same index
  if idx == last_inverted_check_idx then
    return
  end

  local item = mission:get_item(idx)
  if not item then
    return
  end

  -- 210 = MAV_CMD_DO_INVERTED_FLIGHT
  if item:command() == 210 then
    local p1 = item:param1() -- Inverted (0=Normal, 1=Inverted)
    gcs:send_text(
      MAV_SEVERITY.WARNING,
      string.format("WP %d: MAV_CMD_DO_INVERTED_FLIGHT detected! Param1=%.1f", idx, p1)
    )
    last_inverted_check_idx = idx
  elseif item:command() ~= 16 then
    -- if it is not nav waypoint we assume we just check once
    last_inverted_check_idx = idx
  end
  -- Note: If it is a NAV command (like Waypoint), we might want to check constantly
  -- but usually DO commands are instantaneous.
  -- For safety, we only log once per index to avoid spam.
end

-------------------------------------------------------------------------------
------------------------- HIGH LEVEL CONTROL FUNCTIONS ------------------------
-------------------------------------------------------------------------------
--[[
Control the actions while not armed
--]]
local function notArmed()
  -- Native disarming must retain ownership (including MOT_SAFE_DISARM).
  releaseControlAllocation()
  loadMotorOutputs()
end

--[[
Perform vehicle control in Manual mode
--]]
local function applyPWMManualMode()
  applyAutopilotControl()
end

--[[
Perform vehicle control in Steering mode
--]]
local function applyPWMSteeringMode()
  if yaw_target_rad == nil then
    applyAutopilotControl()
    return
  end
  -- If the pilot or failsafe switched modes, stop pursuing yaw alignment
  if vehicle:get_mode() ~= DRIVING_MODES.STEERING then
    yaw_pid:resetInternalState()
    return
  end

  -- Timeout safety
  yaw_align_steps = yaw_align_steps + 1
  if yaw_align_steps > YAW_ALIGN_MAX_STEPS then
    gcs:send_text(MAV_SEVERITY.WARNING, "Yaw align TIMEOUT -> HOLD")
    applyControlAllocation(0, 0)
    yaw_pid:resetInternalState()
    yaw_target_rad, yaw_target_deg = nil, nil
    vehicle:set_mode(DRIVING_MODES.HOLD)
    return
  end

  -- Current yaw from AHRS (rad) and error to target
  local yaw = ahrs:get_yaw()
  if not funcs:isFinite(yaw) then
    applyControlAllocation(0, 0)
    yaw_target_rad, yaw_target_deg = nil, nil
    yaw_pid:resetInternalState()
    vehicle:set_mode(DRIVING_MODES.HOLD)
    return
  end
  local err = funcs:yawErrorRad(yaw, yaw_target_rad)

  -- Check if we reached the target yaw
  if math.abs(err) <= YAW_THRESH_RAD then
    applyControlAllocation(0, 0)
    yaw_pid:resetInternalState()
    gcs:send_text(
      MAV_SEVERITY.INFO,
      string.format("Yaw aligned (target %.1f deg) -> AUTO", yaw_target_deg)
    )
    yaw_target_rad, yaw_target_deg = nil, nil
    vehicle:set_mode(DRIVING_MODES.AUTO)
    return
  end

  -- Rotate in place with pid output
  local pid_out = yaw_pid:compute(err, UPDATE_DT)
  if math.abs(pid_out) < YAW_DEADBAND then
    pid_out = 0
  end
  applyControlAllocation(0, pid_out)
end

-------------------------------------------------------------------------------
-------------------------------- MAIN LOOP ------------------------------------
-------------------------------------------------------------------------------
local function update()
  -- Safety check for vehicle type
  if param:get('SCR_USER5') ~= 2 then
    releaseControlAllocation()
    resetMissionYawState()
    if not vehicle_type_warning then
      gcs:send_text(MAV_SEVERITY.WARNING, 'Allocation inactive: waiting for SCR_USER5=2')
      vehicle_type_warning = true
    end
    return update, 1000
  end
  vehicle_type_warning = false
  -- Getting SCR_USER params to PID values
  local p, i, d = (param:get('SCR_USER2') or 0) / 1000,
    (param:get('SCR_USER3') or 0) / 1000, (param:get('SCR_USER4') or 0) / 1000
  yaw_pid:setGains(p, i, d)

  -- Run not armed routine to guarantee trim values
  if not arming:is_armed() then
    resetMissionYawState()
    notArmed()
    return update, 200
  end

  -- Raw channel overrides bypass native PWM conversion, including E-Stop.
  if SRV_Channels:get_emergency_stop() then
    resetMissionYawState()
    releaseControlAllocation()
    return update, UPDATE_PERIOD_MS
  end

  if not act_obs_avoid_bound then
    act_obs_avoid_bound = act_obs_avoid_param:init('BATT_SOC_ALLOC')
      or act_obs_avoid_param:init('BATT_SOC_ACTOBSAVD') -- previous system-start.lua
  end
  if not act_obs_avoid_bound or act_obs_avoid_param:get() ~= 1 then
    resetMissionYawState()
    releaseControlAllocation()
    return update, UPDATE_PERIOD_MS
  end

  if not outputs_loaded and not loadMotorOutputs() then
    return update, 1000
  end
  local mode = vehicle:get_mode()
  if mode ~= DRIVING_MODES.STEERING and yaw_target_rad ~= nil then
    resetMissionYawState()
  end

  if mode == DRIVING_MODES.MANUAL then
    applyPWMManualMode()
    return update, UPDATE_PERIOD_MS
  elseif mode == DRIVING_MODES.STEERING then
    applyPWMSteeringMode()
    return update, UPDATE_PERIOD_MS
  elseif mode == DRIVING_MODES.HOLD then
    -- Preserve the autopilot's active braking instead of forcing neutral.
    releaseControlAllocation()
    return update, UPDATE_PERIOD_MS
  elseif mode == DRIVING_MODES.AUTO then
    update_active_roi_from_mission()

    local idx = mission:get_current_nav_index()

    -- Detect mission restart / rewind: current index went backwards
    if idx and last_nav_idx and idx < last_nav_idx then
      gcs:send_text(MAV_SEVERITY.INFO, "Mission restart detected -> resetting yaw state")
      resetMissionYawState()
    end

    -- Controls end of mission
    local mission_state = mission:state()
    if mission_state == MISSION_STATE.FINISHED then
      applyControlAllocation(0, 0)
      vehicle:set_mode(DRIVING_MODES.HOLD)
      return update, UPDATE_PERIOD_MS
    end

    -- Allocation must not interpret the usual param4=0 as a mandatory north
    -- alignment at every waypoint. Enable this separate maneuver explicitly.
    if not waypoint_yaw_bound then
      waypoint_yaw_bound = waypoint_yaw_param:init('BATT_SOC_WPYAW')
    end
    if waypoint_yaw_bound and waypoint_yaw_param:get() == 1 then
      logCurrentWpParam4()
      if triggerYawControlOnReachedWaypoint() then
        return update, UPDATE_PERIOD_MS
      end
    else
      last_nav_idx = idx
    end

    -- Check for Inverted Flight Command (210)
    -- checkForInvertedFlightCommand()

    -- Atualiza orientação do gimbal para o ROI, no máximo a 1 Hz
    local now_ms = millis()
    if last_mount_update_ms == nil or (now_ms - last_mount_update_ms):toint() > 1000 then
      send_mount_control_to_roi()
      last_mount_update_ms = now_ms
    end

    -- Acquiring throttle and steering from internal control output
    applyAutopilotControl()
    return update, UPDATE_PERIOD_MS
  end
  -- GUIDED, RTL and other native modes must not terminate this callback or
  -- retain the last raw PWM command from MANUAL/custom AUTO.
  resetMissionYawState()
  releaseControlAllocation()
  return update, UPDATE_PERIOD_MS
end

return update, 3000 -- allow system-start.lua to register dynamic parameters
-------------------------------------------------------------------------------
