-- Minimal ArduPilot scripting mock + skid-steer yaw plant
package.path = '../scripts/modules/?.lua;' .. package.path
local P = { SCR_USER5 = 2, SERVO1_TRIM = 1500, RC1_TRIM = 1500, RC3_TRIM = 1500, RC3_REVERSED = 0,
  SCR_USER2 = 0, SCR_USER3 = 0 }
local S = { mode = 4, armed = true, sw = 0, yaw = 0, rate = 0, pwm = {}, nav = 1, mstate = 1, items = {}, log = {} }
local PLANT = { K = tonumber(os.getenv("K") or 2), dz = 0.15, tau = tonumber(os.getenv("TAU") or 0.3) }
_G.SIM = S
param = { get = function(_, n) return P[n] end, set_and_save = function(_, n, v) P[n] = v; return true end,
  add_table = function() return true end, add_param = function() return true end, set = function(_, n, v) P[n] = v; return true end }
gcs = { send_text = function(_, sev, msg) print(string.format("[%7.2fs] GCS: %s", S.t or 0, msg)) end }
rc = { get_pwm = function() return 1500 end, get_aux_cached = function() return S.sw end }
arming = { is_armed = function() return S.armed end }
vehicle = { get_mode = function() return S.mode end, set_mode = function(_, m) S.mode = m; return true end,
  get_control_output = function() return 0 end }
ahrs = { get_yaw = function() return S.yaw end }
SRV_Channels = { set_output_pwm_chan_timeout = function(_, ch, pwm) S.pwm[ch] = pwm end }
mission = { get_current_nav_index = function() return S.nav end, state = function() return S.mstate end,
  get_item = function(_, i) return S.items[i] end }

local function plant(dt)
  local u = ((S.pwm[1] or 1500) - 1500) / 450  -- left side motor command
  local ue = math.abs(u) > PLANT.dz and (u - PLANT.dz * (u > 0 and 1 or -1)) or 0
  S.rate = S.rate + dt * (PLANT.K * ue - S.rate) / PLANT.tau
  S.yaw = (S.yaw + S.rate * dt + math.pi) % (2 * math.pi) - math.pi
end

-- load script
local chunk = assert(loadfile('../scripts/rover-controlallocation.lua'))
local fn, delay = chunk()
S.t = 0
local next_t = delay / 1000
local function run(until_t, hook)
  while S.t < until_t do
    if S.t >= next_t then
      local f, d = fn()
      assert(f, "SCRIPT DIED (returned nil) in mode " .. S.mode)
      fn = f; next_t = S.t + d / 1000
    end
    plant(0.005); S.t = S.t + 0.005
    if hook and hook() then return end
  end
end
return { S = S, P = P, run = run }
