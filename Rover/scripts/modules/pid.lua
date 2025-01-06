-- Check if the script is being run directly by APM, if so exit
if ... == nil then
  return
end

local PID = {}
PID.__index = PID

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

function PID:setGains(p_gain, i_gain, d_gain)
  self.P = tonumber(p_gain)
  self.I = tonumber(i_gain)
  self.D = tonumber(d_gain)
end

function PID:limitRange(value, min, max)
  return math.min(math.max(value, min), max)
end

function PID:resetInternalState()
  self.integrator = 0
  self.last_error = nil
end

function PID:compute(setpoint, current_value, dt)
  local error = setpoint - current_value
  local deriv = (error - self.last_error) / dt
  self.integrator = self.integrator + error * dt
  self.integrator = self:limitRange(self.integrator, self.i_min, self.i_max)

  local pid_value = self.P * error + self.I * self.integrator + self.D * deriv
  local output = self:limitRange(pid_value, self.pid_min, self.pid_max)

  self.last_error = error

  return output
end

function PID:compute_debug(error, dt)
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
  self.integrator = self:limitRange(self.integrator + error * dt, self.i_min, self.i_max)
  local integrator_output = self.I * self.integrator

  -- PID control signal
  local pid_value = proportional_output + integrator_output + derivative_output
  local pid_output = self:limitRange(pid_value, self.pid_min, self.pid_max)

  return proportional_output, integrator_output, derivative_output, pid_output
end

return PID
