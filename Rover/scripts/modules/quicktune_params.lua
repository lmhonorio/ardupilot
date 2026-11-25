--[[
QuickTune parameters (RTUN_). Loaded at boot so they stay in the GCS
parameter list; the QuickTune logic (quicktune.lua) loads only while it is
used (SCR_USER1 = 1 or its RC switch) and is unloaded afterwards to save
scripting memory.
--]]
if ... == nil then
  return
end

local QP = {}

local PARAM_TABLE_KEY = 15
local PARAM_TABLE_PREFIX = "RTUN_"
local PARAM_TABLE_SIZE = 13

-- bind a parameter to a variable
local function bind_param(name)
   local p = Parameter()
   assert(p:init(name), string.format("RTun: could not find %s parameter", name))
   return p
end

-- add a parameter and bind it to a variable
local function bind_add_param(name, idx, default_value)
   assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format("RTun: could not add param %s", name))
   return bind_param(PARAM_TABLE_PREFIX .. name)
end

-- setup quicktune specific parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, PARAM_TABLE_SIZE), "RTun: could not add param table")

--[[
  // @Param: RTUN_ENABLE
  // @DisplayName: Rover Quicktune enable
  // @Description: Enable quicktune system
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
QP.ENABLE = bind_add_param('ENABLE', 1, 1)

--[[
  // @Param: RTUN_AXES
  // @DisplayName: Rover Quicktune axes
  // @Description: axes to tune
  // @Bitmask: 0:Steering,1:Speed
  // @User: Standard
--]]
QP.AXES = bind_add_param('AXES', 2, 3)

--[[
  // @Param: RTUN_STR_FFRATIO
  // @DisplayName: Rover Quicktune Steering Rate FeedForward ratio
  // @Description: Ratio between measured response and FF gain. Raise this to get a higher FF gain
  // @Range: 0 1.0
  // @User: Standard
--]]
QP.STR_FFRATIO = bind_add_param('STR_FFRATIO', 3, 0.9)

--[[
  // @Param: RTUN_STR_P_RATIO
  // @DisplayName: Rover Quicktune Steering FF to P ratio
  // @Description: Ratio between steering FF and P gains. Raise this to get a higher P gain, 0 to leave P unchanged
  // @Range: 0 2.0
  // @User: Standard
--]]
QP.STR_P_RATIO = bind_add_param('STR_P_RATIO', 4, 0.5)

--[[
  // @Param: RTUN_STR_I_RATIO
  // @DisplayName: Rover Quicktune Steering FF to I ratio
  // @Description: Ratio between steering FF and I gains. Raise this to get a higher I gain, 0 to leave I unchanged
  // @Range: 0 2.0
  // @User: Standard
--]]
QP.STR_I_RATIO = bind_add_param('STR_I_RATIO', 5, 0.5)

--[[
  // @Param: RTUN_SPD_FFRATIO
  // @DisplayName: Rover Quicktune Speed FeedForward (equivalent) ratio
  // @Description: Ratio between measured response and CRUISE_THROTTLE value. Raise this to get a higher CRUISE_THROTTLE value
  // @Range: 0 1.0
  // @User: Standard
--]]
QP.SPD_FFRATIO = bind_add_param('SPD_FFRATIO', 6, 1.0)

--[[
  // @Param: RTUN_SPD_P_RATIO
  // @DisplayName: Rover Quicktune Speed FF to P ratio
  // @Description: Ratio between speed FF and P gain. Raise this to get a higher P gain, 0 to leave P unchanged
  // @Range: 0 2.0
  // @User: Standard
--]]
QP.SPD_P_RATIO = bind_add_param('SPD_P_RATIO', 7, 1.0)

--[[
  // @Param: RTUN_SPD_I_RATIO
  // @DisplayName: Rover Quicktune Speed FF to I ratio
  // @Description: Ratio between speed FF and I gain. Raise this to get a higher I gain, 0 to leave I unchanged
  // @Range: 0 2.0
  // @User: Standard
--]]
QP.SPD_I_RATIO = bind_add_param('SPD_I_RATIO', 8, 1.0)

--[[
  // @Param: RTUN_AUTO_FILTER
  // @DisplayName: Rover Quicktune auto filter enable
  // @Description: When enabled the PID filter settings are automatically set based on INS_GYRO_FILTER
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
QP.AUTO_FILTER = bind_add_param('AUTO_FILTER', 9, 1)

--[[
  // @Param: RTUN_AUTO_SAVE
  // @DisplayName: Rover Quicktune auto save
  // @Description: Number of seconds after completion of tune to auto-save. This is useful when using a 2 position switch for quicktune
  // @Units: s
  // @User: Standard
--]]
QP.AUTO_SAVE = bind_add_param('AUTO_SAVE', 10, 5)

--[[
  // @Param: RTUN_RC_FUNC
  // @DisplayName: Rover Quicktune RC function
  // @Description: RCn_OPTION number to use to control tuning stop/start/save
  // @Values: 300:Scripting1, 301:Scripting2, 302:Scripting3, 303:Scripting4, 304:Scripting5, 305:Scripting6, 306:Scripting7, 307:Scripting8
  // @User: Standard
--]]
QP.RC_FUNC = bind_add_param('RC_FUNC', 11, 300)

--[[
  // @Param: RTUN_SPEED_MIN
  // @DisplayName: Rover Quicktune minimum speed for tuning
  // @Description: The mimimum speed in m/s required for tuning to start
  // @Units: m/s
  // @Range: 0.1 0.5
  // @User: Standard
--]]
QP.SPEED_MIN = bind_add_param('SPEED_MIN', 12, 0.5)

--[[
  // @Param: RTUN_MSG
  // @DisplayName: Rover Quicktune messages
  // @Description: 0 shows only the overall progress (Tune NN%) plus saved / cancelled. 1 shows every message: stages, live values, stalls with hints, and each gain change.
  // @Values: 0:Progress only,1:Full
  // @User: Standard
--]]
QP.MSG = bind_add_param('MSG', 13, 0)

return QP
