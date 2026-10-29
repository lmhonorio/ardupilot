-- Run from the repository root: lua Rover/tests/test_control_allocation.lua
-- No firmware, network, PWM device or vehicle is accessed by these tests.
package.path = 'Rover/scripts/modules/?.lua;' .. package.path
local funcs = require('functions')
local count = 0
local function equal(actual, expected)
  assert(actual == expected, string.format('expected %s, got %s', tostring(expected), tostring(actual)))
end
local function near(actual, expected)
  assert(math.abs(actual - expected) < 1e-9, string.format('expected %.9g, got %.9g', expected, actual))
end
local function test(name, body)
  body()
  count = count + 1
  print('OK ' .. name)
end

test('inclusive clamp does not turn minimum into maximum', function()
  equal(funcs:mapMaxMin(0.1, 0.1, 1), 0.1)
  equal(funcs:mapMaxMin(1050, 1050, 1950), 1050)
  equal(funcs:mapMaxMin(-2, -1, 1), -1)
end)

test('mixed commands retain steering authority without saturation', function()
  local left, right = funcs:allocateRightAndLeft(0.4, 0.2)
  near(left, 0.6); near(right, 0.2)
  left, right = funcs:allocateRightAndLeft(-0.4, 0.2)
  near(left, -0.2); near(right, -0.6)
  left, right = funcs:allocateRightAndLeft(0, 0.5)
  near(left, 0.5); near(right, -0.5)
end)

test('saturation preserves side ratio and reverse symmetry', function()
  local left, right = funcs:allocateRightAndLeft(1, 0.5)
  near(left, 1); near(right, 1 / 3)
  for ti = -20, 20 do
    for si = -20, 20 do
      local t, s = ti / 20, si / 20
      left, right = funcs:allocateRightAndLeft(t, s)
      local neg_left, neg_right = funcs:allocateRightAndLeft(-t, -s)
      assert(math.abs(left) <= 1 and math.abs(right) <= 1)
      near(neg_left, -left); near(neg_right, -right)
      near(left * (t - s), right * (t + s))
    end
  end
end)

test('recorded 44 percent throttle and 59 percent steering reaches the expected outputs', function()
  local left, right = funcs:allocateRightAndLeft(0.4383198547, 0.585625271)
  equal(funcs:normalizedToPwm(left, 1000, 1500, 2000, false), 2000)
  equal(funcs:normalizedToPwm(right, 1000, 1500, 2000, false), 1428)
end)

test('saturated mix respects throttle or steering priority', function()
  local left, right = funcs:allocateRightAndLeft(0.8, 0.6, 0)
  near(left, 1); near(right, 0.6)
  left, right = funcs:allocateRightAndLeft(0.8, 0.6, 1)
  near(left, 1); near(right, -0.2)
  left, right = funcs:allocateRightAndLeft(-0.8, -0.6, 0)
  near(left, -1); near(right, -0.6)
end)

test('asymmetric reverse thrust is corrected within actuator limits', function()
  local left, right = funcs:allocateRightAndLeft(0, 1, 0.5, 2)
  near(left, 0.5); near(right, -1)
  for ti = -10, 10 do
    for si = -10, 10 do
      for _, priority in ipairs({0, 0.5, 1}) do
        left, right = funcs:allocateRightAndLeft(ti / 10, si / 10, priority, 2)
        assert(math.abs(left) <= 1 and math.abs(right) <= 1)
      end
    end
  end
end)

test('motor curve preserves neutral and applies minimum symmetrically', function()
  near(funcs:scaleMotorOutput(0, 10, 0.5), 0)
  near(funcs:scaleMotorOutput(0.4, 10, 0), 0.46)
  near(funcs:scaleMotorOutput(-0.4, 10, 0), -0.46)
  local output = funcs:scaleMotorOutput(0.3, 0, 0.6)
  near(0.4 * output + 0.6 * output * output, 0.3)
end)

test('invalid commands produce neutral, never one moving side', function()
  for _, bad in ipairs({0 / 0, math.huge, -math.huge, 'invalid'}) do
    local left, right = funcs:allocateRightAndLeft(bad, 0.3)
    equal(left, 0); equal(right, 0)
  end
  local left, right = funcs:allocateRightAndLeft(nil, 0.3)
  equal(left, 0); equal(right, 0)
end)

test('per-channel PWM uses asymmetric spans, trim and reversal', function()
  equal(funcs:normalizedToPwm(0, 1100, 1520, 1950, true), 1520)
  equal(funcs:normalizedToPwm(0.5, 1100, 1520, 1950, false), 1735)
  equal(funcs:normalizedToPwm(0.5, 1100, 1520, 1950, true), 1310)
  equal(funcs:normalizedToPwm(-1, 1100, 1520, 1950, false), 1100)
  equal(funcs:normalizedToPwm(0, 1500, 1500, 2000, false), nil)
  local left, right = funcs:allocateRightAndLeftPwmShare(-0.4, 0, 450)
  equal(left, -180); equal(right, -180)
end)

test('angle wrap handles more than one revolution', function()
  equal(funcs:mapTo360(-720), 0)
  equal(funcs:mapTo360(810), 90)
end)

local function fixture()
  local state = {armed = true, mode = 10, throttle = 0, steering = 0,
    index = 1, mission_state = 1, yaw = 0, clock = 0, writes = {}, active = {}, modes = {},
    params = {SCR_USER5 = 2, SCR_USER2 = 25, SCR_USER3 = 1, SCR_USER4 = 4,
      MOT_PWM_TYPE = 0, BATT_SOC_ALLOC = 1, BATT_SOC_WPYAW = 0}, items = {}}
  function state:servo(number, fn, minimum, trim, maximum, reversed)
    local prefix = 'SERVO' .. number .. '_'
    self.params[prefix .. 'FUNCTION'] = fn
    self.params[prefix .. 'MIN'] = minimum or 1000
    self.params[prefix .. 'TRIM'] = trim or 1500
    self.params[prefix .. 'MAX'] = maximum or 2000
    self.params[prefix .. 'REVERSED'] = reversed or 0
  end
  state:servo(1, 73); state:servo(3, 74)
  param = {get = function(_, name) return state.params[name] end}
  Parameter = function()
    return {init = function(self, name)
      if state.params[name] == nil then return false end
      self.name = name
      return true
    end, get = function(self)
      assert(self.name, 'get on an unbound Parameter')
      return state.params[self.name]
    end}
  end
  gcs = {send_text = function() end}
  arming = {is_armed = function() return state.armed end}
  vehicle = {get_mode = function() return state.mode end,
    set_mode = function(_, mode) state.mode = mode; state.modes[#state.modes + 1] = mode; return true end,
    get_control_output = function(_, channel)
      if channel == 3 then return state.throttle end
      return state.steering
    end}
  SRV_Channels = {get_emergency_stop = function() return state.emergency_stop or false end,
    set_output_pwm_chan_timeout = function(_, channel, pwm, timeout)
    assert(pwm >= 1000 and pwm <= 2000 and pwm == math.floor(pwm))
    state.writes[#state.writes + 1] = {channel, pwm, timeout}
    state.active[channel] = timeout > 0 and pwm or nil
  end}
  local item = {command = function() return 16 end, param4 = function() return 0 end,
    z = function() return 77 end}
  mission = {get_current_nav_index = function() return state.index end,
    state = function() return state.mission_state end,
    get_item = function(_, index) return state.items[index] or item end}
  ahrs = {get_yaw = function() return state.yaw end, get_location = function() return nil end}
  local time_meta = {}
  time_meta.__index = {toint = function(self) return self.value end}
  time_meta.__sub = function(a, b) return setmetatable({value = a.value - b.value}, time_meta) end
  millis = function() return setmetatable({value = state.clock}, time_meta) end
  state.callback = assert(loadfile('Rover/scripts/rover-controlallocation.lua'))()
  function state:step()
    self.clock = self.clock + 20
    local callback, delay = self.callback()
    equal(type(callback), 'function')
    self.callback = callback
    return delay
  end
  return state
end

test('AUTO zero throttle is zero, and braking/reverse stay signed', function()
  local f = fixture()
  equal(f:step(), 20)
  equal(f.active[0], 1500); equal(f.active[2], 1500)
  f.throttle = -0.4; f:step()
  equal(f.active[0], 1300); equal(f.active[2], 1300)
  f.throttle = -1; f:step()
  equal(f.active[0], 1000); equal(f.active[2], 1000)
end)

test('MANUAL uses autopilot RC processing and affects only assigned motors', function()
  local f = fixture(); f.mode = 0; f.throttle = 0.4; f.steering = 0.2
  f:step()
  equal(f.active[0], 1800); equal(f.active[2], 1600)
  equal(f.active[1], nil); equal(f.active[3], nil)
end)

test('allocation reads motor mixing and curve parameters', function()
  local f = fixture(); f.throttle = 0.8; f.steering = 0.6
  f.params.MOT_STR_THR_MIX = 1
  f:step()
  equal(f.active[0], 2000); equal(f.active[2], 1400)
  f.throttle = 0.4; f.steering = 0
  f.params.MOT_THR_MIN = 10
  f:step()
  equal(f.active[0], 1730); equal(f.active[2], 1730)
end)

test('late vehicle configuration does not terminate the driver', function()
  local f = fixture(); f.params.SCR_USER5 = 0
  equal(f:step(), 1000); equal(f.active[0], nil)
  f.params.SCR_USER5 = 2; f.throttle = 0.4
  f:step(); equal(f.active[0], 1700)
end)

test('wheel rate control retains native actuator ownership', function()
  local f = fixture(); f.params.WRC_ENABLE = 1; f.throttle = 0.4
  f:step(); equal(f.active[0], nil); equal(f.active[2], nil)
end)

test('four motors follow SERVO functions, individual trims and polarity', function()
  local f = fixture(); f.throttle = 0.4
  f:servo(1, 74, 1100, 1550, 1900, 1)
  f:servo(2, 73, 1050, 1460, 1960, 0)
  f:servo(3, 73, 1000, 1500, 2000, 1)
  f:servo(4, 74, 1000, 1500, 2000, 0)
  f:step()
  equal(f.active[0], 1370); equal(f.active[1], 1660)
  equal(f.active[2], 1300); equal(f.active[3], 1700)
end)

test('disarming and HOLD immediately release raw PWM overrides', function()
  local f = fixture(); f.throttle = 0.5; f:step()
  f.mode = 4; f:step()
  equal(next(f.active), nil)
  f.mode = 0; f:step(); equal(f.active[0], 1750)
  f.armed = false; f:step(); equal(next(f.active), nil)
end)

test('GUIDED executes controller outputs without changing mode', function()
  local f = fixture(); f.mode = 15; f.throttle = 0.4; f.steering = 0.2
  equal(f:step(), 20); equal(f.active[0], 1800); equal(f.active[2], 1600)
  equal(#f.modes, 0)
end)

test('E-Stop releases an existing override and blocks new commands', function()
  local f = fixture(); f.throttle = 0.5; f:step()
  f.emergency_stop = true; f:step(); equal(next(f.active), nil)
  local writes = #f.writes
  f:step(); equal(#f.writes, writes)
end)

test('brushed or digital motor protocols are left to the native driver', function()
  local f = fixture(); f.params.MOT_PWM_TYPE = 3
  equal(f:step(), 1000); equal(next(f.active), nil)
end)

test('disabling custom AUTO immediately releases previous commands', function()
  local f = fixture(); f.throttle = 0.4; f:step()
  f.params.BATT_SOC_ALLOC = 0; f:step()
  equal(next(f.active), nil)
end)

test('dynamic parameter can be created after the script is loaded', function()
  local f = fixture(); f.params.BATT_SOC_ALLOC = nil; f:step()
  equal(next(f.active), nil)
  f.params.BATT_SOC_ALLOC = 1; f.throttle = 0.2; f:step()
  equal(f.active[0], 1600)
end)

test('allocator disable also preserves native MANUAL control', function()
  local f = fixture(); f.mode = 0; f.params.BATT_SOC_ALLOC = 0; f.throttle = 1
  f:step(); equal(next(f.active), nil)
end)

test('STEERING without a yaw maneuver does not force HOLD or change modes', function()
  local f = fixture(); f.mode = 3; f.steering = 0.3; f:step()
  equal(f.mode, 3); equal(#f.modes, 0)
  equal(f.active[0], 1650); equal(f.active[2], 1350)
end)

test('waypoints and mission state do not change modes or override controller', function()
  local f = fixture(); f.params.BATT_SOC_WPYAW = 1
  f.throttle = 0.2; f.steering = 0.1; f:step()
  f.index = 2; f.mission_state = 2; f:step()
  equal(f.mode, 10); equal(#f.modes, 0)
  equal(f.active[0], 1650); equal(f.active[2], 1550)
end)

test('missing controller measurement commands neutral on both sides', function()
  local f = fixture(); f.throttle = nil; f.steering = 0.8; f:step()
  equal(f.active[0], 1500); equal(f.active[2], 1500)
end)

test('invalid servo calibration prevents allocation', function()
  local f = fixture(); f:servo(3, 74, 1500, 1500, 2000, 0)
  equal(f:step(), 1000); equal(next(f.active), nil)
end)

print(string.format('%d tests passed', count))
