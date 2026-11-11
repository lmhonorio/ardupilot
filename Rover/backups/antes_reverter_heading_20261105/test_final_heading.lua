package.path='scripts/modules/?.lua;'..package.path
local nav=require('heading_final')
local function near(a,b,e) assert(math.abs(a-b)<e,string.format('%.5f != %.5f',a,b)) end
local function item(cmd,lat,lon,frame)
 return {command=function()return cmd end,frame=function()return frame or 3 end,
         x=function()return lat or 0 end,y=function()return lon or 0 end}
end
local items={[1]=item(16,-31231500,-417644950),[2]=item(16,-31231500,-417644750),[3]=item(92)}
local dest,angle=nav.endpoints(4,function(i)return items[i] end)
assert(dest.index==2);near(angle,math.pi/2,.001)
local steer,error=nav.steer(0,angle,0);assert(steer>0 and error>0)
steer,error=nav.steer(math.pi/2,angle,0);assert(steer==0)
items[3]=item(20);assert(nav.endpoints(4,function(i)return items[i] end)==nil)
items[3]=item(177);assert(nav.endpoints(4,function(i)return items[i] end)==nil)

local values={SCR_USER5=2,BATT_SOC_ALLOC=1,MOT_PWM_TYPE=0,WRC_ENABLE=0,
  SERVO1_FUNCTION=73,SERVO1_MIN=1000,SERVO1_TRIM=1500,SERVO1_MAX=2000,SERVO1_REVERSED=0,
  SERVO3_FUNCTION=74,SERVO3_MIN=1000,SERVO3_TRIM=1500,SERVO3_MAX=2000,SERVO3_REVERSED=0,
  MOT_STR_THR_MIX=.5,MOT_THST_ASYM=1,MOT_THR_MIN=0,MOT_THST_EXPO=0,WP_RADIUS=.3,PRX1_TYPE=2,OA_TYPE=1}
param={get=function(_,n)return values[n] end}
function Parameter() return {init=function()return true end,get=function()return 1 end} end
local armed=false;arming={is_armed=function()return armed end}
local mode=10;local throttle,turn=.2,.3
vehicle={get_mode=function()return mode end,get_control_output=function(_,n)if n==3 then return throttle else return turn end end}
local state=1;items[3]=item(92)
mission={MISSION_COMPLETE=2,state=function()return state end,num_commands=function()return 4 end,get_item=function(_,i)return items[i] end}
local yaw=0;local outputs={}
SRV_Channels={set_output_pwm_chan_timeout=function(_,ch,pwm,timeout)outputs[ch]={pwm=pwm,timeout=timeout} end,get_emergency_stop=function()return false end}
local status=2;local object=2
proximity={get_status=function()return status end,get_closest_object=function()return 0,object end}
local loc={lat=function()return -31231500 end,lng=function()return -417644750 end}
local velocity={length=function()return 0 end};local gyro={z=function()return 0 end}
ahrs={healthy=function()return true end,get_position=function()return loc end,groundspeed_vector=function()return velocity end,get_gyro=function()return gyro end,get_yaw=function()return yaw end}
gcs={send_text=function()end}
local update=assert(loadfile('scripts/rover-controlallocation.lua'))()
update();armed=true;update()
assert(outputs[0].pwm>outputs[2].pwm,'native steering must pass during mission')
state=2;status=1;update()
assert(outputs[0].timeout==0,'missing proximity must block final pivot')
status=2;update()
assert(outputs[0].pwm>1500 and outputs[2].pwm<1500,'final heading rotates toward last segment')
object=.3;update()
assert(outputs[0].timeout==0,'obstacle must interrupt pivot')
mode=4;update()
mode=10;state=1;object=2;update()
state=2;throttle=0;turn=0;update()
yaw=math.pi/2
for _=1,50 do update() end
assert(outputs[0].timeout==0,'aligned vehicle must release motor overrides')
print('Lua: geometria, repasse da navegacao, alinhamento e bloqueio de obstaculo OK')
