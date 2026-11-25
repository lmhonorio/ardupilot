--[[
Rover: the only script in APM/scripts. Everything else is a module in
APM/scripts/modules, loaded here and run by one scheduler:

  task       module          period
  mixer      mixer.lua       20-200 ms (per mode)
  quicktune  quicktune.lua   25 ms tuning, 100 ms idle
  can        can_esc.lua     5 ms

Output modules (remove one to drop that path):
  out_pwm.lua                                      PWM on MAIN 1..4
  can_esc.lua + can_state, can_speed, can_wheeltune, can_report   DJI C620 on CAN
Other modules: rover_common, gain_banks, yaw_align, yaw_tune, circle-tune,
autotune, pid, functions.

A fault in one task is reported and handled there; the others keep running.
The old stand-alone scripts (esc_can.lua, rover-controlallocation.lua,
rover-quicktune.lua) must not be in APM/scripts with this one: two copies
would drive the same motors. If one is found, nothing starts and arming is refused.
--]]

local MAV_SEVERITY = { EMERGENCY = 0, ALERT = 1, CRITICAL = 2, ERROR = 3, WARNING = 4, NOTICE = 5, INFO = 6, DEBUG = 7 }
local VERSION = "r2611a"

local OLD_SCRIPTS = { "esc_can.lua", "rover-controlallocation.lua", "rover-quicktune.lua" }
local SCRIPT_DIRS = { "APM/scripts/", "scripts/" }

local function find_old_script()
  for _, dir in ipairs(SCRIPT_DIRS) do
    for _, name in ipairs(OLD_SCRIPTS) do
      local f = io.open(dir .. name, "rb")
      if f ~= nil then
        f:close()
        return dir .. name
      end
    end
  end
  return nil
end

local old = find_old_script()
if old ~= nil then
  local auth = arming:get_aux_auth_id()
  local function refuse()
    gcs:send_text(MAV_SEVERITY.CRITICAL, "Rover: delete " .. old .. "; nothing started")
    if auth ~= nil then
      arming:set_aux_auth_failed(auth, "delete old " .. string.sub(old, -28))
    end
    return refuse, 5000
  end
  return refuse()
end

-- Modules. A missing output module only drops that path.
local R = require("rover_common")

-- "file.lua:123: message" without the folder, so the GCS line keeps the useful part
local function short_error(err)
  local text = tostring(err)
  return string.match(text, "([%w_%-]+%.lua:%d+:.*)$") or text
end

local function load_module(name, required)
  local ok, result = pcall(require, name)
  if ok then
    return result
  end
  local text = tostring(result)
  if not required and string.find(text, "not found", 1, true) then
    -- Removed on purpose: that output path is simply off
    gcs:send_text(MAV_SEVERITY.INFO, string.format("Rover: %s not installed", name))
  else
    gcs:send_text(MAV_SEVERITY.CRITICAL, string.format("Rover: %s failed: %s", name,
      string.sub(short_error(text), 1, 36)))
  end
  return nil
end

load_module("out_pwm", false)
local mixer = load_module("mixer", true)
local quicktune = load_module("quicktune", true)
local can = load_module("can_esc", false)
if R.pwm == nil and R.can == nil then
  gcs:send_text(MAV_SEVERITY.CRITICAL, "Rover: no output module (out_pwm or can_esc)")
end

--[[
Scheduler. Each task: step() returns the delay to its next run in ms, or nil
to stop. The tasks run in a fixed order when due: the mixer's command reaches
the CAN driver in the same cycle.
--]]
local auth_id = nil
local tasks = {}
local function add_task(name, module, on_fault)
  if module == nil then
    return
  end
  tasks[#tasks + 1] = { name = name, step = module.step, due = module.FIRST_DELAY_MS or 0,
    on_fault = on_fault, fault = nil, warn_ms = -5000 }
end

add_task("mixer", mixer, function()
  -- A mixer fault stops the wheels and refuses arming until reboot
  mixer.stop_outputs()
  if auth_id == nil then
    auth_id = arming:get_aux_auth_id()
  end
  if auth_id ~= nil then
    arming:set_aux_auth_failed(auth_id, "mixer fault; reboot")
  end
  return 100
end)
add_task("quicktune", quicktune, nil)
add_task("can", can, nil)

local function run_task(t, now)
  if t.fault ~= nil then
    -- Faulted: keep the safe action going and remind the operator every 5 s
    if now - t.warn_ms >= 5000 then
      t.warn_ms = now
      gcs:send_text(MAV_SEVERITY.CRITICAL, string.format("Rover: %s fault: %s", t.name, string.sub(short_error(t.fault), 1, 34)))
    end
    local ok, delay = pcall(t.on_fault)
    t.due = now + ((ok and delay) or 100)
    return
  end
  local ok, delay = pcall(t.step)
  if not ok then
    t.fault = tostring(delay)
    if t.on_fault == nil then
      -- Tasks without a fault action handle their own errors; this one is unexpected
      gcs:send_text(MAV_SEVERITY.CRITICAL, string.format("Rover: %s stopped: %s", t.name, string.sub(short_error(t.fault), 1, 34)))
      t.dead = true
      return
    end
    t.warn_ms = -5000
    run_task(t, now)
    return
  end
  if delay == nil then
    t.dead = true
    return
  end
  t.due = now + delay
end

local function update()
  local now = R.now_ms()
  local next_due = nil
  for i = 1, #tasks do
    local t = tasks[i]
    if not t.dead then
      if now >= t.due then
        run_task(t, now)
      end
      if not t.dead and (next_due == nil or t.due < next_due) then
        next_due = t.due
      end
    end
  end
  if next_due == nil then
    gcs:send_text(MAV_SEVERITY.WARNING, "Rover: no task left")
    return
  end
  return update, math.max(1, math.floor(next_due - now))
end

gcs:send_text(MAV_SEVERITY.INFO, string.format("Rover %s: PWM %s, CAN %s", VERSION,
  R.pwm and "yes" or "no", R.can and "yes" or "no"))

return update()
