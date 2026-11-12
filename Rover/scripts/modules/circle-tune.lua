if ... == nil then
  return
end

local Autotune = require("autotune")
local CircleTune = {}
CircleTune.__index = CircleTune
CircleTune.PERIOD_MS = 25

local PARAM_KEY = 18
local CIRCLE_MODE = 9
local GAIN_P = "ATC_STR_RAT_P"
local GAIN_I = "ATC_STR_RAT_I"
local SETTLE_S = 3
local WAIT_S = 60
local DEFAULT_AUTO_SAVE_S = 5
local MIN_SPEED = 0.3
local MIN_RATE = math.rad(3)
local MAX_RATE_ERROR = math.rad(12)
local MAX_COMMAND = 0.95
local MAX_NATIVE_STEERING_CHANGE = 0.15

local function finite(value)
  return type(value) == "number" and math.abs(value) < math.huge
end

local function message(severity, text)
  gcs:send_text(severity, "CircleTune: " .. text)
end

-- esc_can.lua publishes ESC_STAT: bit 0 = CAN active, bit 1 = current saturated.
-- Never set with PWM, so this check only acts on the CAN path.
local ESC_STAT_CAN_SATURATED = 1 | 2
local esc_stat = Parameter()
local esc_stat_bound = false
local esc_stat_retry_ms = 0

local function esc_saturated()
  if not esc_stat_bound then
    local now = millis():toint()
    if now < esc_stat_retry_ms then
      return false
    end
    esc_stat_retry_ms = now + 2000
    esc_stat_bound = esc_stat:init("ESC_STAT")
    if not esc_stat_bound then
      return false
    end
  end
  return (math.floor(esc_stat:get() or 0) & ESC_STAT_CAN_SATURATED) == ESC_STAT_CAN_SATURATED
end

function CircleTune:new()
  assert(param:add_table(PARAM_KEY, "CTUN_", 2), "could not add CircleTune parameter table")
  assert(param:add_param(PARAM_KEY, 1, "RELAY", 5), "could not add CTUN_RELAY")
  assert(param:add_param(PARAM_KEY, 2, "RAD_ERR", 2), "could not add CTUN_RAD_ERR")
  return setmetatable({
    status = "idle",
    relay = Autotune:new({
      hyst_rad = math.rad(1),
      max_amp_rad = MAX_RATE_ERROR,
      min_period_s = 0.5,
      min_amp_ratio = 2,
    }),
  }, self)
end

function CircleTune:cancel(reason)
  if self.status == "pending_save" then
    if param:get(GAIN_P) == self.applied_p and not param:set(GAIN_P, self.original_p) then
      message(2, "could not restore steering P")
    end
    if param:get(GAIN_I) == self.applied_i and not param:set(GAIN_I, self.original_i) then
      message(2, "could not restore steering I")
    end
  end
  if self.status == "waiting" or self.status == "running" or self.status == "pending_save" then
    self.status = "aborted"
    message(4, "aborted - " .. reason)
  end
end

function CircleTune:reset(reason)
  if self.status ~= "idle" then
    self:cancel(reason)
    self.status = "idle"
    self.baseline = nil
    self.had_rc_input = false
  end
end

local function outputs_enabled()
  return arming:is_armed() and not SRV_Channels:get_emergency_stop() and
         not SRV_Channels:get_safety_state()
end

function CircleTune:measure()
  if vehicle:get_mode() ~= CIRCLE_MODE or not ahrs:healthy() or
     (battery and battery:has_failsafed()) or not ahrs:get_relative_position_NED_origin() then
    return
  end
  if rc:has_valid_input() then
    self.had_rc_input = true
    local steer = rc:get_channel(param:get("RCMAP_ROLL"))
    local throttle = rc:get_channel(param:get("RCMAP_THROTTLE"))
    if (steer and steer:norm_input_dz() ~= 0) or
       (throttle and throttle:norm_input_dz() ~= 0) then
      return
    end
  elseif self.had_rc_input then
    return
  end

  local velocity = ahrs:get_velocity_NED()
  local radial_error = vehicle:get_wp_crosstrack_error_m()
  if not velocity or not finite(radial_error) then
    return
  end
  local yaw, pitch, roll = ahrs:get_yaw(), ahrs:get_pitch(), ahrs:get_roll()
  local gyro = ahrs:get_gyro()
  local rate = -math.sin(pitch) * gyro:x() +
               math.cos(pitch) * (math.sin(roll) * gyro:y() + math.cos(roll) * gyro:z())
  local speed = math.cos(yaw) * velocity:x() + math.sin(yaw) * velocity:y()
  return rate, speed, math.abs(radial_error)
end

function CircleTune:finish(now)
  if not outputs_enabled() or vehicle:get_mode() ~= CIRCLE_MODE then
    self:cancel("Circle state unavailable before applying gains")
    return
  end
  local result = self.relay:result()
  if not finite(result.P) or not finite(result.I) or
     result.P <= 0 or result.P > 2 or result.I <= 0 or result.I > 2 then
    self:cancel("candidate gains outside parameter range")
    return
  end

  if param:get(GAIN_P) ~= self.original_p or param:get(GAIN_I) ~= self.original_i then
    self:cancel("steering gains changed during the test")
    return
  end
  if not param:set(GAIN_P, result.P) then
    self:cancel("could not apply steering P")
    return
  end
  if not param:set(GAIN_I, result.I) then
    if not param:set(GAIN_P, self.original_p) then
      message(2, "could not restore steering P")
    end
    self:cancel("could not apply steering I")
    return
  end

  local save_delay = param:get("RTUN_AUTO_SAVE")
  self.save_delay = finite(save_delay) and save_delay or DEFAULT_AUTO_SAVE_S
  if self.save_delay > 0 then
    -- Read back the gains at the firmware's parameter precision.
    self.applied_p = param:get(GAIN_P)
    self.applied_i = param:get(GAIN_I)
    self.applied_at = now
    self.status = "pending_save"
  else
    self.status = "done"
  end
  message(5, string.format("P=%.3f I=%.3f applied in RAM; Ku=%.2f Pu=%.2fs",
    result.P, result.I, result.Ku, result.Pu))
end

function CircleTune:save_gains()
  if param:get("SCR_USER1") ~= 3 or not outputs_enabled() or vehicle:get_mode() ~= CIRCLE_MODE then
    self:cancel("Circle tune deactivated before saving")
    return
  end
  if param:get(GAIN_P) ~= self.applied_p or param:get(GAIN_I) ~= self.applied_i then
    self:cancel("steering gains changed before saving")
    return
  end
  if not param:set_and_save(GAIN_P, self.applied_p) then
    self:cancel("could not save steering P")
    return
  end
  if not param:set_and_save(GAIN_I, self.applied_i) then
    if not param:set_and_save(GAIN_P, self.original_p) then
      message(2, "could not restore saved steering P")
    end
    self:cancel("could not save steering I")
    return
  end

  self.status = "done"
  message(5, "tuning gains saved")
  if param:get("SCR_USER1") == 3 then
    if param:set_and_save("SCR_USER1", 0) then
      message(5, "SCR_USER1 set to 0 (normal)")
    else
      message(4, "gains saved; could not reset SCR_USER1")
    end
  end
end

function CircleTune:sample_baseline(now, rate, speed, steering)
  local b = self.baseline
  if not b or math.abs(rate - b.rate) > math.rad(1.5) or
     math.abs(speed - b.speed) > math.max(0.10, 0.10 * b.speed) or
     math.abs(steering - b.steering) > 0.03 then
    self.baseline = { start = now, rate = rate, speed = speed, steering = steering,
      sum_rate = 0, sum_speed = 0, sum_steering = 0, count = 0 }
    b = self.baseline
  end
  b.count = b.count + 1
  b.sum_rate = b.sum_rate + rate
  b.sum_speed = b.sum_speed + speed
  b.sum_steering = b.sum_steering + steering
  if (now - b.start):tofloat() * 0.001 >= SETTLE_S then
    self.rate_ref = b.sum_rate / b.count
    self.speed_ref = b.sum_speed / b.count
    self.steering_base = b.sum_steering / b.count
    self.original_p = param:get(GAIN_P)
    self.original_i = param:get(GAIN_I)
    if not finite(self.original_p) or not finite(self.original_i) then
      self:cancel("steering gains unavailable")
      return
    end
    self.relay:start()
    self.status = "running"
    message(5, string.format("relay started at %.1f deg/s, %.2f m/s, steering %.2f",
      math.deg(self.rate_ref), self.speed_ref, self.steering_base))
  end
end

-- A nil command returns steering to the native PID. The second result allows PWM writes.
function CircleTune:update(throttle, steering, pwm_range_pct)
  if not vehicle.get_wp_crosstrack_error_m or not SRV_Channels.get_safety_state or
     not SRV_Channels.get_emergency_stop then
    if self.status == "idle" then
      self.status = "aborted"
      message(3, "Circle cross-track or safety API unavailable")
    end
    return nil, finite(throttle) and finite(steering)
  end

  local allow_output = outputs_enabled() and finite(throttle) and finite(steering)
  if self.status == "done" or self.status == "aborted" then
    return nil, allow_output
  end

  local now = millis()
  local rate, speed, radial_error
  if allow_output then
    rate, speed, radial_error = self:measure()
  end

  if self.status == "idle" then
    local relay_pct = param:get("CTUN_RELAY")
    self.max_radial_error = param:get("CTUN_RAD_ERR")
    self.status = "waiting"
    if not finite(relay_pct) or relay_pct < 1 or relay_pct > 20 or
       not finite(self.max_radial_error) or self.max_radial_error <= 0 then
      self:cancel("invalid CTUN_RELAY or CTUN_RAD_ERR")
      return nil, allow_output
    end
    self.relay.relay = relay_pct * 0.01
    self.wait_start = now
    message(5, "waiting for stable Circle")
  end

  local dt = self.last_sample and (now - self.last_sample):tofloat() * 0.001 or 0
  self.last_sample = now
  local room = allow_output and throttle > 0 and throttle < MAX_COMMAND and
               finite(pwm_range_pct) and pwm_range_pct > 0
  local tracking = finite(rate) and finite(speed) and finite(radial_error) and
                   speed >= MIN_SPEED and radial_error <= self.max_radial_error
  if self.status == "pending_save" then
    if not tracking then
      self:cancel("Circle tracking outside limits before saving")
    elseif (now - self.applied_at):tofloat() * 0.001 >= self.save_delay then
      self:save_gains()
    end
    return nil, allow_output
  end
  if self.status == "waiting" then
    if (now - self.wait_start):tofloat() * 0.001 >= WAIT_S then
      self:cancel("no stable Circle operating point")
    elseif room and tracking and math.abs(rate) >= MIN_RATE and
           math.abs(steering) > self.relay.relay and math.abs(steering) + self.relay.relay < MAX_COMMAND then
      if dt >= 0.05 then
        self.baseline = nil
      end
      self:sample_baseline(now, rate, speed, steering)
    else
      self.baseline = nil
    end
    return nil, allow_output
  end

  if not tracking or not room or dt <= 0 or dt >= 0.05 or
     math.abs(speed - self.speed_ref) > math.max(0.15, 0.20 * self.speed_ref) then
    self:cancel("Circle tracking, speed, output or timing outside limits")
    return nil, allow_output
  end
  -- The firmware PID keeps running while Lua replaces its output at the mixer.
  if math.abs(steering - self.steering_base) > MAX_NATIVE_STEERING_CHANGE then
    self:cancel("native steering diverged from baseline")
    return nil, allow_output
  end
  if esc_saturated() then
    self:cancel("wheel current saturated (ESC_CUR)")
    return nil, allow_output
  end

  local delta, status = self.relay:step(self.rate_ref - rate, dt)
  if status == "done" then
    self:finish(now)
    return nil, allow_output
  end
  if status == "fail" then
    self:cancel(self.relay:result().msg)
    return nil, allow_output
  end

  self.rate, self.speed, self.radial_error = rate, speed, radial_error
  return self.steering_base + delta, true
end

function CircleTune:log_sample(throttle, steering, pwm_l, pwm_r)
  if logger then
    logger:write("CRTU", "Target,Rate,Speed,RadErr,Throt,Steer,PWML,PWMR,Cycle", "ffffffHHB",
      self.rate_ref, self.rate, self.speed, self.radial_error, throttle, steering, pwm_l, pwm_r, self.relay.cycle)
  end
end

return CircleTune
