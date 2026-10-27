local P = { SCR_USER6 = 1, RTUN_ENABLE=1, RTUN_AXES=3, RTUN_RC_FUNC=300, RTUN_AUTO_SAVE=5, RTUN_AUTO_FILTER=1 }
setmetatable(P,{__index=function() return 0.5 end})
local mode, now, msgs = 0, 0, {}
function Parameter(n) local o={name=n}; function o:init(x) self.name=x; return true end
  function o:get() return P[self.name] end; function o:set(v) P[self.name]=v; return true end
  function o:set_and_save(v) P[self.name]=v; return true end; return o end
param={add_table=function() return true end, add_param=function(_,_,_,n,v) return true end,
  get=function(_,n) return P[n] end, set=function(_,n,v) P[n]=v end, set_and_save=function(_,n,v) P[n]=v end}
gcs={send_text=function(_,_,m) print(string.format("[%5.1fs] %s",now/1000,m)) end}
millis=function() return {toint=function() return now end, tofloat=function() return now end} end
uint32_t=function(x) return x end
local mt={__index=function(t,k) return function() return 0 end end}
rc=setmetatable({get_channel=function() return {norm_input_dz=function() return 0 end} end, get_aux_cached=function() return nil end, get_pwm=function() return 1500 end},mt)
vehicle={get_mode=function() return mode end, get_steering_and_throttle=function() return 0.3,0.4 end}
arming={is_armed=function() return true end}
local v3={x=function() return 1.0 end,y=function() return 0 end,z=function() return 0.5 end}
ahrs=setmetatable({get_gyro=function() return v3 end,get_velocity_NED=function() return v3 end,earth_to_body=function() return v3 end},mt)
logger={write=function() end}
AR_AttitudeControl={get_srate=function() return 0,0 end}
local fn=assert(loadfile('../scripts/rover-quicktune.lua'))
local f,d=fn()
local function run(ms) local e=now+ms; while now<e do f,d=f(); now=now+d end end
print("-- MANUAL"); run(2000)
print("-- CIRCLE"); mode=9; run(60000)
print("SCR_USER6 after tune =",P.SCR_USER6, "ATC_STR_RAT_FF=",P.ATC_STR_RAT_FF, "ATC_SPEED_FF=",P.ATC_SPEED_FF)
P.SCR_USER6=1; mode=0; run(1000); mode=9; run(3000); print("-- leave CIRCLE mid-tune"); mode=0; run(1000)
print("SCR_USER6 =",P.SCR_USER6)
