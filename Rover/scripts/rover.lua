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

Modules load one per script run. ArduPilot collects all Lua garbage after
each run, so the compiler's scratch memory of one module is freed before the
next one.

On demand (the scripting heap could not hold everything; can_wheeltune did not
load with "not enough mem"):
  quicktune.lua      loads with SCR_USER1 = 1 (or its RTUN_RC_FUNC switch),
                     unloads once the selector leaves 1 and no gain is pending.
                     Its RTUN_ parameters (quicktune_params.lua) load at boot.
  can_wheeltune.lua  loads with SCR_USER1 = 4 or 5, unloads once the selector
                     leaves 4/5 and the tune is idle.

require() takes the script environment from the FIRST upvalue of the function
ArduPilot is running (load_next, then update). Both name a global first so
that upvalue is _ENV.

A fault in one task is reported and handled there; the others keep running.
The old stand-alone scripts (esc_can.lua, rover-controlallocation.lua,
rover-quicktune.lua) must not be in APM/scripts with this one: two copies
would drive the same motors. If one is found, nothing starts and arming is refused.
--]]

local MAV_SEVERITY = { EMERGENCY = 0, ALERT = 1, CRITICAL = 2, ERROR = 3, WARNING = 4, NOTICE = 5, INFO = 6, DEBUG = 7 }
local VERSION = "r2611f"

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

local R = require("rover_common")

-- "file.lua:123: message" without the folder, so the GCS line keeps the useful part
local function short_error(err)
  local text = tostring(err)
  return string.match(text, "([%w_%-]+%.lua:%d+:.*)$") or text
end

-- A GCS line holds 50 characters: send a long text over several lines,
-- breaking at spaces (at most 6 lines)
local function send_long(severity, text)
  local LINE = 48
  local lines = 0
  while #text > 0 and lines < 6 do
    local chunk = text
    if #text > LINE then
      local cut = LINE
      for i = LINE, 20, -1 do
        if string.sub(text, i, i) == " " then
          cut = i
          break
        end
      end
      chunk = string.sub(text, 1, cut)
    end
    gcs:send_text(severity, (string.gsub(chunk, "%s+$", "")))
    text = string.sub(text, #chunk + 1)
    lines = lines + 1
  end
end

--[[
Load order. kind:
  "core"  needed; a failure is critical
  "pwm"   PWM output; missing = PWM off
  "can"   CAN driver; missing can_state = CAN off and the other can_* are skipped
  "canopt" optional part of the CAN driver
A module's own require() of one already loaded costs nothing.
--]]
local LOAD = {
  { "functions", "core" }, { "pid", "core" }, { "autotune", "core" }, { "circle-tune", "core" },
  { "out_pwm", "pwm" },
  { "gain_banks", "core" }, { "yaw_align", "core" }, { "yaw_tune", "core" }, { "mixer", "core" },
  { "quicktune_params", "core" },
  { "can_state", "can" }, { "can_speed", "can" }, { "can_esc", "can" },
  { "can_report", "canopt" },
}
local MANAGE_MS = 500
local LOAD_STEP_MS = 20

local loaded = {}
local can_off = false
-- Boot report: module names by result. Dropped after the boot summary;
-- later (on-demand) loads report themselves.
local report = { ok = {}, failed = {}, missing = {} }

--[[
Lua memory. A script cannot read it (no collectgarbage), but ArduPilot sends
"Lua: Time: <us> Mem: <total> + <run>" after a run while SCR_DEBUG_OPTS bit 1
is set; <total> - <run> is the memory in use when the run started.
mem_report() sets the bit for one run only and restores the saved value at
the start of the next run, so exactly one such line goes out. It is asked one
run after a load or unload, so the start of the measured run already holds
the new module set after a full collection.
esc_monitor.py turns it into a percentage of SCR_HEAP_SIZE.
--]]
local DEBUG_RUNTIME_MSG = 2
local debug_opts = Parameter()
local debug_bound = debug_opts:init('SCR_DEBUG_OPTS')
local debug_restore = nil
-- Ask on the next run, so the reported memory includes what this run loaded
local mem_pending = false

local function mem_report()
  if not debug_bound or debug_restore ~= nil then
    return
  end
  local v = math.floor(debug_opts:get() or 0)
  if (v & DEBUG_RUNTIME_MSG) ~= 0 then
    return -- already reported after every run
  end
  debug_restore = v
  debug_opts:set(v | DEBUG_RUNTIME_MSG)
end

-- Start of every run: undo the one-run report request
local function mem_report_done()
  if debug_restore ~= nil then
    debug_opts:set(debug_restore)
    debug_restore = nil
  end
end

-- Returns true when the module is loaded
local function load_one(name, kind)
  if (kind == "can" or kind == "canopt") and can_off then
    return false
  end
  local ok, result = pcall(require, name)
  if ok then
    loaded[name] = result
    if report ~= nil then
      report.ok[#report.ok + 1] = name
    end
    return true
  end
  local text = tostring(result)
  local missing = string.find(text, "module '" .. name .. "' not found", 1, true) ~= nil
  if kind == "can" then
    can_off = true
  end
  if missing and kind ~= "core" then
    -- Removed on purpose: that path or option is simply off
    gcs:send_text(MAV_SEVERITY.INFO, string.format("Rover: %s not installed", name))
    if report ~= nil then
      report.missing[#report.missing + 1] = name
    end
    return false
  end
  -- The reason is at the end ("...:\n\t<reason>" for a load error)
  local reason = string.match(text, "\t([^\t]*)$") or short_error(text)
  gcs:send_text(MAV_SEVERITY.CRITICAL, string.format("Rover: %s failed:", name))
  send_long(MAV_SEVERITY.CRITICAL, short_error(reason))
  if report ~= nil then
    report.failed[#report.failed + 1] = name
  end
  return false
end

-- Boot summary: modules loaded, failed and not installed, then one memory line
local function send_report()
  local sev = (#report.failed > 0) and MAV_SEVERITY.CRITICAL or MAV_SEVERITY.INFO
  gcs:send_text(sev, string.format("Rover Lua: %d OK, %d falha, %d ausente",
    #report.ok, #report.failed, #report.missing))
  send_long(MAV_SEVERITY.INFO, "OK: " .. table.concat(report.ok, " "))
  if #report.failed > 0 then
    send_long(MAV_SEVERITY.CRITICAL, "FALHA: " .. table.concat(report.failed, " "))
  end
  if #report.missing > 0 then
    send_long(MAV_SEVERITY.INFO, "Ausente: " .. table.concat(report.missing, " "))
  end
  gcs:send_text(MAV_SEVERITY.INFO, "Sob demanda: quicktune (USER1=1), wheeltune (4/5)")
  gcs:send_text(MAV_SEVERITY.INFO, string.format("Memoria: heap %.0f kB; uso na linha Lua: Mem",
    (param:get('SCR_HEAP_SIZE') or 0) / 1024))
  report = nil
  mem_pending = true
end

-- Drop a module so the next collection frees it
local function unload(name)
  loaded[name] = nil
  package.loaded[name] = nil
end

--[[
Scheduler. Each task: step() returns the delay to its next run in ms, or nil
to stop. The tasks run in a fixed order when due: the mixer's command reaches
the CAN driver in the same cycle.
--]]
local auth_id = nil
local tasks = {}
local function add_task(name, module, start_ms, on_fault)
  if module == nil then
    return
  end
  tasks[#tasks + 1] = { name = name, step = module.step, due = start_ms + (module.FIRST_DELAY_MS or 0),
    on_fault = on_fault, fault = nil, warn_ms = -5000 }
end

local function remove_task(name)
  for i = #tasks, 1, -1 do
    if tasks[i].name == name then
      table.remove(tasks, i)
    end
  end
end

--[[
On-demand modules, checked every MANAGE_MS. Unload first, then at most one
load per check, so each compile has the heap after a full collection.
--]]
local selector_param = Parameter()
local selector_bound = false
local manage_due = 0
local wt_failed = false
local qt_failed = false
local load_warn_ms = -5000

local function quicktune_wanted(sel)
  local qp = loaded["quicktune_params"]
  if qp == nil or (qp.ENABLE:get() or 0) <= 0 then
    return false
  end
  if sel == R.TUNE_MODE.PIXHAWK then
    return true
  end
  local sw = rc:get_aux_cached(qp.RC_FUNC:get())
  return sw ~= nil and sw ~= 0
end

local function wheeltune_wanted(sel)
  return R.can ~= nil and (sel == R.TUNE_MODE.WHEEL_GROUND or sel == R.TUNE_MODE.WHEEL_AIR)
end

local function manage(now)
  if not selector_bound then
    selector_bound = selector_param:init('SCR_USER1')
    if not selector_bound then
      return
    end
  end
  local sel = selector_param:get() or 0
  local qt = loaded["quicktune"]
  local wt = loaded["can_wheeltune"]
  if qt ~= nil and not quicktune_wanted(sel) and qt.idle() then
    remove_task("quicktune")
    unload("quicktune")
    qt = nil
    gcs:send_text(MAV_SEVERITY.INFO, "Rover: quicktune unloaded")
    mem_pending = true
  end
  if wt ~= nil and not wheeltune_wanted(sel) and wt.idle() then
    wt.unload()
    unload("can_wheeltune")
    wt = nil
    gcs:send_text(MAV_SEVERITY.INFO, "Rover: wheel tune unloaded")
    mem_pending = true
  end
  -- A failed load is not retried until the selector leaves that tune
  if not quicktune_wanted(sel) then
    qt_failed = false
  end
  if not wheeltune_wanted(sel) then
    wt_failed = false
  end
  -- Compiling a tune takes over a second (quicktune: 1.4 s on the board) and
  -- the 5 ms CAN loop waits for it: load only while disarmed. Both tunes
  -- start on arming, so this costs nothing in normal use.
  local want_load = (qt == nil and not qt_failed and quicktune_wanted(sel)) or
    (wt == nil and not wt_failed and wheeltune_wanted(sel))
  -- Not in a run that reports memory: the line would include the compile
  if want_load and debug_restore ~= nil then
    return
  end
  if want_load and arming:is_armed() then
    if now - load_warn_ms >= 5000 then
      load_warn_ms = now
      gcs:send_text(MAV_SEVERITY.WARNING, "Rover: disarm to load the tune selected in SCR_USER1")
    end
    return
  end
  if qt == nil and not qt_failed and quicktune_wanted(sel) then
    if load_one("quicktune", "core") then
      add_task("quicktune", loaded["quicktune"], now, nil)
      mem_pending = true
    else
      qt_failed = true
    end
    return
  end
  if wt == nil and not wt_failed and wheeltune_wanted(sel) then
    if load_one("can_wheeltune", "canopt") then
      gcs:send_text(MAV_SEVERITY.INFO, "Rover: wheel tune loaded")
      mem_pending = true
    else
      wt_failed = true
    end
  end
end

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
  -- First upvalue must be _ENV: manage() may require() a module in this run
  local _ = require
  mem_report_done()
  if mem_pending then
    mem_pending = false
    mem_report()
  end
  local now = R.now_ms()
  if now >= manage_due then
    manage_due = now + MANAGE_MS
    manage(now)
  end
  local next_due = manage_due
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

local start_ms = R.now_ms()

local function start_tasks()
  local mixer = loaded["mixer"]
  if R.pwm == nil and R.can == nil then
    gcs:send_text(MAV_SEVERITY.CRITICAL, "Rover: no output module (out_pwm or can_esc)")
  end
  add_task("mixer", mixer, start_ms, function()
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
  add_task("can", R.can, start_ms, nil)
  gcs:send_text(MAV_SEVERITY.INFO, string.format("Rover %s: PWM %s, CAN %s", VERSION,
    R.pwm and "yes" or "no", R.can and "yes" or "no"))
  send_report()
  -- Next run, not this one: update() starts by clearing the memory request,
  -- and ArduPilot sends the memory line after this run ends
  return update, 1
end

-- One module per run, then the scheduler takes over
local load_index = 0
local function load_next()
  -- ArduPilot's require() takes the script environment from the FIRST
  -- upvalue of the function it is running now. Name a global first so that
  -- upvalue is _ENV; with load_index first, every require failed with
  -- "attempt to index a number value".
  local _ = require
  load_index = load_index + 1
  local item = LOAD[load_index]
  if item == nil then
    return start_tasks()
  end
  load_one(item[1], item[2])
  return load_next, LOAD_STEP_MS
end

return load_next()
