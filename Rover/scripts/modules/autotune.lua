-- Check if the script is being run directly by APM, if so exit
if ... == nil then
  return
end

--[[
Relay-feedback autotune (Astrom-Hagglund) + Tyreus-Luyben PI.

Used for:
  - in-place yaw alignment (error in rad, relay in steering)
  - inner wheel-speed loop (error in rpm, relay in amperes)

Usage:
  local Autotune = require("autotune")
  local at = Autotune:new({ relay = 0.45, hyst = math.rad(2) })
  at:start()
  local u, status = at:step(error, dt)  -- status: "running" | "done" | "fail"
  local res = at:result()  -- { P, I, Ku, Pu, amp, msg }
--]]

local Autotune = {}
Autotune.__index = Autotune

local DEFAULTS = {
  relay = 0.45,               -- relay amplitude d (same units as the plant input)
  hyst = math.rad(2),         -- relay hysteresis epsilon (same units as error)
  skip_cycles = 2,            -- initial cycles discarded (transient)
  cycles = 4,                 -- cycles used for the estimate
  max_amp = math.rad(45),     -- abort if |error| grows beyond this
  timeout_s = 60,             -- abort if not finished in time
  min_period_s = 0.4,         -- sanity limits for Pu
  max_period_s = 20,
  max_spread = 0.25,          -- max relative deviation of Pu/amp between cycles
  kp_div = 3.2,               -- Tyreus-Luyben: Kp = Ku / 3.2
  ti_mul = 2.2,               -- Tyreus-Luyben: Ti = 2.2 * Pu
}

function Autotune:new(cfg)
  local obj = {}
  for k, v in pairs(DEFAULTS) do
    obj[k] = (cfg and cfg[k] ~= nil) and cfg[k] or v
  end
  -- Back-compat with the yaw script names
  if cfg then
    if cfg.hyst_rad ~= nil then
      obj.hyst = cfg.hyst_rad
    end
    if cfg.max_amp_rad ~= nil then
      obj.max_amp = cfg.max_amp_rad
    end
  end
  obj.hyst_rad = obj.hyst
  obj.max_amp_rad = obj.max_amp
  setmetatable(obj, self)
  obj:reset()
  return obj
end

function Autotune:reset()
  self.t = 0
  self.u = 0
  self.last_rise_t = nil
  self.e_max = -math.huge
  self.e_min = math.huge
  self.cycle = 0
  self.periods = {}
  self.amps = {}
  self.status = "idle"
  self.res = nil
end

function Autotune:start()
  self:reset()
  self.status = "running"
end

local function mean(t)
  local s = 0
  for _, v in ipairs(t) do s = s + v end
  return s / #t
end

local function maxRelDev(t, m)
  local d = 0
  for _, v in ipairs(t) do d = math.max(d, math.abs(v - m) / m) end
  return d
end

function Autotune:fail(msg)
  self.status = "fail"
  self.res = { msg = msg }
  return 0, self.status
end

function Autotune:finish()
  local Pu = mean(self.periods)
  local a = mean(self.amps)
  if maxRelDev(self.periods, Pu) > self.max_spread or maxRelDev(self.amps, a) > self.max_spread then
    return self:fail("inconsistent cycles")
  end
  if Pu < self.min_period_s or Pu > self.max_period_s then
    return self:fail(string.format("Pu out of range (%.2fs)", Pu))
  end
  if a <= self.hyst then
    return self:fail("amplitude below hysteresis, increase relay")
  end
  local Ku = 4 * self.relay / (math.pi * math.sqrt(a * a - self.hyst * self.hyst))
  local Kp = Ku / self.kp_div
  local Ti = self.ti_mul * Pu
  self.status = "done"
  self.res = { P = Kp, I = Kp / Ti, Ku = Ku, Pu = Pu, amp = a, msg = "ok" }
  return 0, self.status
end

--[[
Run one step of the relay experiment
-- @param err number - control error (target - measured)
-- @param dt number - time since the last step in s
-- @return number, string - plant input and status
--]]
function Autotune:step(err, dt)
  if self.status ~= "running" then
    return 0, self.status
  end
  self.t = self.t + dt
  if self.t > self.timeout_s then
    return self:fail("timeout")
  end
  if math.abs(err) > self.max_amp then
    return self:fail("oscillation too large")
  end

  self.e_max = math.max(self.e_max, err)
  self.e_min = math.min(self.e_min, err)

  -- Relay with hysteresis
  local new_u = self.u
  if err > self.hyst then
    new_u = self.relay
  elseif err < -self.hyst then
    new_u = -self.relay
  elseif self.u == 0 then
    new_u = self.relay
  end

  -- One full period is measured between consecutive switches to +d
  if new_u > 0 and self.u <= 0 then
    if self.last_rise_t ~= nil then
      self.cycle = self.cycle + 1
      if self.cycle > self.skip_cycles then
        table.insert(self.periods, self.t - self.last_rise_t)
        table.insert(self.amps, (self.e_max - self.e_min) / 2)
      end
    end
    self.last_rise_t = self.t
    self.e_max = -math.huge
    self.e_min = math.huge
  end
  self.u = new_u

  if #self.periods >= self.cycles then
    return self:finish()
  end
  return self.u, self.status
end

function Autotune:result()
  return self.res
end

return Autotune
