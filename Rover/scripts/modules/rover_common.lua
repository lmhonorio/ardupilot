--[[
Shared constants, helpers and the context table of the Rover scripts.

Every module gets the same table from require("rover_common"): require()
caches a module, so this is the one place where modules find each other.
  R.pwm   PWM output (out_pwm.lua), nil when that module is not installed
  R.can   CAN driver (can_esc.lua), nil when that module is not installed
--]]
if ... == nil then
  return
end

local R = {}

R.MAV_SEVERITY = { EMERGENCY = 0, ALERT = 1, CRITICAL = 2, ERROR = 3, WARNING = 4, NOTICE = 5, INFO = 6, DEBUG = 7 }
R.DRIVING_MODES = { MANUAL = 0, ACRO = 1, STEERING = 3, HOLD = 4, LOITER = 5, SIMPLE = 7, CIRCLE = 9, AUTO = 10,
  RTL = 11, SMART_RTL = 12, GUIDED = 15 }
R.MISSION_STATE = { IDLE = 0, RUNNING = 1, FINISHED = 2 }

-- SCR_USER1 selector values
R.TUNE_MODE = { NORMAL = 0, PIXHAWK = 1, YAW = 2, CIRCLE = 3, WHEEL_GROUND = 4, WHEEL_AIR = 5 }

R.PWM_TRIM_VALUE = 1500

-- Output modules fill these in when they load
R.pwm = nil
R.can = nil

-- Rover 4.7 gives millis() and micros() as uint32_t. tonumber() of that value is nil.
function R.as_number(value)
  if type(value) == "number" then
    return value
  end
  if value ~= nil and value.tofloat ~= nil then
    return value:tofloat()
  end
  return tonumber(value)
end

function R.now_ms()
  return R.as_number(millis())
end

function R.constrain(value, lo, hi)
  if value > hi then
    return hi
  elseif value < lo then
    return lo
  end
  return value
end

function R.finite(value)
  return type(value) == "number" and value == value and math.abs(value) < math.huge
end

-- True when the CAN driver is installed and drives the wheels
function R.can_selected()
  return R.can ~= nil and R.can.selected()
end

-- With ESC_CAN = 1 no pulse may reach MAIN 1..4: it drops the C620 out of CAN
function R.pwm_allowed()
  return R.can == nil or not R.can.pwm_blocked()
end

return R
