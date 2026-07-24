-- Rover control allocation - DO_SET_REVERSE diagnostic version
--
-- Purpose:
--   1. Keep the rover in AUTO without automatic mode changes.
--   2. Read raw ArduRover throttle/yaw outputs before any clamp.
--   3. Preserve negative throttle values in the custom allocation.
--   4. Inspect the mission and report the last DO_SET_REVERSE command.
--
-- Required SD-card module:
--   APM/scripts/modules/functions.lua

-- @param control_output integer - CONTROL MODE 4
-- | '1' # Roll
-- | '2' # Pitch
-- | '3' # Throttle
-- | '4' # Yaw
-- | '5' # Lateral
-- | '6' # MainSail
-- | '7' # WingSail
-- | '8' # Walking_Height
-- @return number|nil

package.path = package.path .. ';./scripts/modules/?.lua'
local funcs = require("functions")

-------------------------------------------------------------------------------
-- CONSTANTS
-------------------------------------------------------------------------------

local UPDATE_PERIOD_MS = 200
local DEBUG_PERIOD_STEPS = 5 -- 5 x 200 ms = approximately 1 second

local THROTTLE_CONTROL_OUTPUT = 3
local YAW_CONTROL_OUTPUT = 4

local MAV_CMD_NAV_WAYPOINT = 16
local MAV_CMD_DO_SET_REVERSE = 194

local MAX_CHANNEL_OUTPUT = 1950
local MIN_CHANNEL_OUTPUT = 1050
local PWM_RANGE = 450

local MAV_SEVERITY = {
    EMERGENCY = 0,
    ALERT = 1,
    CRITICAL = 2,
    ERROR = 3,
    WARNING = 4,
    NOTICE = 5,
    INFO = 6,
    DEBUG = 7
}

local DRIVING_MODES = {
    MANUAL = 0,
    STEERING = 3,
    HOLD = 4,
    AUTO = 10,
    GUIDED = 15
}

local MISSION_STATE = {
    IDLE = 0,
    RUNNING = 1,
    FINISHED = 2
}

-------------------------------------------------------------------------------
-- PARAMETERS AND STATE
-------------------------------------------------------------------------------

local VEHICLE_TYPE = tonumber(param:get('SCR_USER5')) or 0
local PWM_TRIM_VALUE = tonumber(param:get('SERVO1_TRIM')) or 1500
local RC1_TRIM_VALUE = tonumber(param:get('RC1_TRIM')) or 1500
local RC3_TRIM_VALUE = tonumber(param:get('RC3_TRIM')) or 1500

local last_manual_throttle = 0
local throttle_accel_rate_thresh = 0.5
local throttle_accel_rate = 0.5

local last_manual_steering = 0
local steering_accel_rate_thresh = 0.6
local steering_accel_rate = 0.6

local radio_type = 0
local debug_counter = 0
local last_reported_nav_idx = -1

-------------------------------------------------------------------------------
-- BASIC HELPERS
-------------------------------------------------------------------------------

local function clamp(value, minimum, maximum)
    if value < minimum then
        return minimum
    elseif value > maximum then
        return maximum
    end
    return value
end

local function boolToInt(value)
    if value then
        return 1
    end
    return 0
end

-------------------------------------------------------------------------------
-- CONTROL ALLOCATION
-------------------------------------------------------------------------------

-- t: throttle from -1.0 to +1.0
-- s: steering from -1.0 to +1.0
local function applyControlAllocation(t, s)
    t = clamp(tonumber(t) or 0, -1.0, 1.0)
    s = clamp(tonumber(s) or 0, -1.0, 1.0)

    -- Rover motor arrangement:
    --
    --     1 ------- 0
    --         ^
    --         | forward
    --     2 ------- 3
    --
    -- Motors 1 and 2 are on the left side.
    -- Motors 0 and 3 are on the right side.
    local pwm_aloc_l, pwm_aloc_r =
        funcs:allocateRightAndLeftPwmShare(t, s, PWM_RANGE)

    local pwm_l = funcs:mapMaxMin(
        PWM_TRIM_VALUE + pwm_aloc_l,
        MIN_CHANNEL_OUTPUT,
        MAX_CHANNEL_OUTPUT
    )

    local pwm_r = funcs:mapMaxMin(
        PWM_TRIM_VALUE - pwm_aloc_r,
        MIN_CHANNEL_OUTPUT,
        MAX_CHANNEL_OUTPUT
    )

    -- Lua output override remains valid for 300 ms.
    -- The update loop refreshes it every 200 ms.
    SRV_Channels:set_output_pwm_chan_timeout(0, pwm_r, 300)
    SRV_Channels:set_output_pwm_chan_timeout(1, pwm_l, 300)
    SRV_Channels:set_output_pwm_chan_timeout(2, pwm_l, 300)
    SRV_Channels:set_output_pwm_chan_timeout(3, pwm_r, 300)
end

-------------------------------------------------------------------------------
-- MISSION DIAGNOSTICS
-------------------------------------------------------------------------------

-- Finds the last DO_SET_REVERSE before the current NAV item.
-- Returns:
--   reverse_expected: false for forward, true for reverse
--   reverse_item_idx: mission index of the command, or -1 if none was found
local function getMissionReverseState(current_nav_idx)
    if current_nav_idx == nil then
        return false, -1
    end

    local start_idx = math.floor(tonumber(current_nav_idx) or 0) - 1

    for index = start_idx, 0, -1 do
        local item = mission:get_item(index)

        if item and item:command() == MAV_CMD_DO_SET_REVERSE then
            local param1 = tonumber(item:param1()) or 0
            return param1 >= 0.5, index
        end
    end

    return false, -1
end

local function getCurrentNavCommand(current_nav_idx)
    if current_nav_idx == nil then
        return -1
    end

    local item = mission:get_item(current_nav_idx)
    if not item then
        return -1
    end

    return tonumber(item:command()) or -1
end

local function reportAutoState(nav_idx, throttle_raw, yaw_raw)
    local nav_cmd = getCurrentNavCommand(nav_idx)
    local reverse_expected, reverse_item_idx = getMissionReverseState(nav_idx)

    -- Keep each message short because MAVLink STATUSTEXT is length-limited.
    gcs:send_text(
        MAV_SEVERITY.INFO,
        string.format(
            "AUTO idx=%d cmd=%d rev=%d do=%d",
            tonumber(nav_idx) or -1,
            nav_cmd,
            boolToInt(reverse_expected),
            reverse_item_idx
        )
    )

    gcs:send_text(
        MAV_SEVERITY.INFO,
        string.format(
            "RAW throttle=%.3f yaw=%.3f",
            throttle_raw,
            yaw_raw
        )
    )
end

-------------------------------------------------------------------------------
-- DISARMED STATE
-------------------------------------------------------------------------------

local function notArmed()
    SRV_Channels:set_output_pwm_chan_timeout(0, PWM_TRIM_VALUE, 3000)
    SRV_Channels:set_output_pwm_chan_timeout(1, PWM_TRIM_VALUE, 3000)
    SRV_Channels:set_output_pwm_chan_timeout(2, PWM_TRIM_VALUE, 3000)
    SRV_Channels:set_output_pwm_chan_timeout(3, PWM_TRIM_VALUE, 3000)
end

-------------------------------------------------------------------------------
-- MANUAL MODE
-------------------------------------------------------------------------------

local function applyPWMManualMode()
    local rc3_pwm = tonumber(rc:get_pwm(3)) or RC3_TRIM_VALUE
    local rc1_pwm = tonumber(rc:get_pwm(1)) or RC1_TRIM_VALUE

    local raw_steering = (rc1_pwm - RC1_TRIM_VALUE) / 450.0
    raw_steering = clamp(raw_steering, -1.0, 1.0)

    local steering = funcs:applyAbsSmoothing(
        raw_steering,
        last_manual_steering,
        steering_accel_rate_thresh,
        steering_accel_rate
    )
    last_manual_steering = steering

    local raw_throttle
    if radio_type == 1 then
        raw_throttle = (rc3_pwm - RC3_TRIM_VALUE) / 450.0
    else
        raw_throttle = (RC3_TRIM_VALUE - rc3_pwm) / 450.0
    end
    raw_throttle = clamp(raw_throttle, -1.0, 1.0)

    local throttle = funcs:applyAbsSmoothing(
        raw_throttle,
        last_manual_throttle,
        throttle_accel_rate_thresh,
        throttle_accel_rate
    )
    last_manual_throttle = throttle

    applyControlAllocation(throttle, steering)
end

-------------------------------------------------------------------------------
-- ARDUPILOT-CONTROLLED MODES
-------------------------------------------------------------------------------

-- Reads the internal Rover control outputs without changing their signs.
local function applyInternalControlOutputs()
    local throttle_raw =
        tonumber(vehicle:get_control_output(THROTTLE_CONTROL_OUTPUT)) or 0

    local yaw_raw =
        tonumber(vehicle:get_control_output(YAW_CONTROL_OUTPUT)) or 0

    throttle_raw = clamp(throttle_raw, -1.0, 1.0)
    yaw_raw = clamp(yaw_raw, -1.0, 1.0)

    applyControlAllocation(throttle_raw, yaw_raw)

    return throttle_raw, yaw_raw
end

-------------------------------------------------------------------------------
-- MAIN LOOP
-------------------------------------------------------------------------------

local function update()
    -- SCR_USER5 must be 2 for Rover in this project.
    if VEHICLE_TYPE ~= 2 then
        gcs:send_text(
            MAV_SEVERITY.WARNING,
            "SCR_USER5 is not 2; Lua outputs stopped"
        )
        applyControlAllocation(0, 0)
        return update, 2000
    end

    radio_type = tonumber(param:get('SCR_USER6')) or 0

    if not arming:is_armed() then
        last_manual_throttle = 0
        last_manual_steering = 0
        debug_counter = 0
        last_reported_nav_idx = -1
        notArmed()
        return update, 2000
    end

    local mode = vehicle:get_mode()

    if mode == DRIVING_MODES.MANUAL then
        applyPWMManualMode()
        return update, UPDATE_PERIOD_MS
    end

    if mode == DRIVING_MODES.HOLD then
        applyControlAllocation(0, 0)
        return update, UPDATE_PERIOD_MS
    end

    if mode == DRIVING_MODES.AUTO then
        local mission_state = mission:state()

        if mission_state == MISSION_STATE.FINISHED then
            -- Do not change mode. Keep the vehicle stopped in AUTO.
            applyControlAllocation(0, 0)
            return update, UPDATE_PERIOD_MS
        end

        local throttle_raw, yaw_raw = applyInternalControlOutputs()
        local nav_idx = mission:get_current_nav_index()

        debug_counter = debug_counter + 1

        -- Report once per second and immediately whenever NAV index changes.
        if debug_counter >= DEBUG_PERIOD_STEPS or nav_idx ~= last_reported_nav_idx then
            debug_counter = 0
            last_reported_nav_idx = nav_idx
            reportAutoState(nav_idx, throttle_raw, yaw_raw)
        end

        return update, UPDATE_PERIOD_MS
    end

    if mode == DRIVING_MODES.STEERING or mode == DRIVING_MODES.GUIDED then
        -- No automatic mode changes are performed in this test version.
        applyInternalControlOutputs()
        return update, UPDATE_PERIOD_MS
    end

    -- Unknown or unsupported mode: command neutral outputs.
    applyControlAllocation(0, 0)
    return update, UPDATE_PERIOD_MS
end

-- Initial delay gives the flight controller time to finish startup.
return update, 3000