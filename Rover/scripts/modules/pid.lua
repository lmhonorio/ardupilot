-- Check if the script is being run directly by APM, if so exit
if ... == nil then
  return
end

local PID = {}
PID.__index = PID

--[[
PID controller class
Usage:
  local pid = PID:new(p_gain, i_gain, d_gain, i_max, i_min, pid_max, pid_min)
  pid:setGains(p_gain, i_gain, d_gain)
  local output = pid:compute(error, dt)
  pid:resetInternalState()
Parameters:
  p_gain (number): Proportional gain
  i_gain (number): Integral gain
  d_gain (number): Derivative gain
  i_max (number): Maximum integrator value
  i_min (number): Minimum integrator value
  pid_max (number): Maximum PID output value
  pid_min (number): Minimum PID output value
--]]
function PID:new(p_gain, i_gain, d_gain, i_max, i_min, pid_max, pid_min)
  local obj = {
    P = p_gain or 0,
    I = i_gain or 0,
    D = d_gain or 0,
    integrator = 0,
    last_error = nil,
    i_max = i_max or 0,
    i_min = i_min or 0,
    pid_max = pid_max or 1,
    pid_min = pid_min or -1
  }
  setmetatable(obj, self)
  -- self.__index = self

  return obj
end

--[[
Set PID gains
-- @param p_gain number - Proportional gain
-- @param i_gain number - Integral gain
-- @param d_gain number - Derivative gain
--]]
function PID:setGains(p_gain, i_gain, d_gain)
  self.P = tonumber(p_gain)
  self.I = tonumber(i_gain)
  self.D = tonumber(d_gain)
end

--[[
Limit the output ranges
-- @param value number - Value to be limited
-- @param min number - Minimum limit
-- @param max number - Maximum limit
-- @return number - Limited value
--]]
function PID:limitRange(value, min, max)
  return math.min(math.max(value, min), max)
end

--[[
Reset the internal state of the PID controller
--]]
function PID:resetInternalState()
  self.integrator = 0
  self.last_error = nil
end

--[[
Compute the PID outputs
-- @param error number - The current error value
-- @param dt number - Time difference since last computation
-- @return number - The computed PID output
--]]
function PID:compute(error, dt)
  -- Proportional output
  local proportional_output = self.P * error
  -- Derivative output
  -- If no last error, dont take into accout the derivative yet
  local derivative_output = 0
  if self.last_error ~= nil then
    derivative_output = self.D * (error - self.last_error) / dt
  end
  self.last_error = error
  -- Integrator output
  self.integrator = self:limitRange(self.integrator + self.I * error * dt, self.i_min, self.i_max)
  local integrator_output = self.integrator

  -- PID control signal
  local pid_value = proportional_output + integrator_output + derivative_output
  local pid_output = self:limitRange(pid_value, self.pid_min, self.pid_max)
  return pid_output
end

return PID
