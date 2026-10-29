"""Prepare an isolated SITL directory; accepts QGC exports or NAME VALUE files."""
import math
from pathlib import Path
import shutil
import sys


def read_parameters(path):
    values = {}
    for number, line in enumerate(Path(path).read_text().splitlines(), 1):
        fields = line.split('#', 1)[0].replace(',', ' ').split()
        if not fields:
            continue
        if len(fields) == 5 and fields[0].isdigit() and fields[1].isdigit():
            name, value = fields[2:4]
        elif len(fields) == 2:
            name, value = fields
        else:
            raise ValueError('%s:%d: expected NAME VALUE or QGC row' % (path, number))
        value = float(value)
        if not name.replace('_', '').isalnum() or name[0].isdigit() or not math.isfinite(value):
            raise ValueError('%s:%d: invalid parameter' % (path, number))
        # These are read-only/system values or truncated dynamic parameter names.
        if name == 'FORMAT_VERSION' or name.startswith('BATT_SOC_'):
            continue
        values[name] = value
    return values


def prepare(rover, run):
    rover, run = Path(rover), Path(run)
    selected = read_parameters(rover / 'gazebo_simple/ros_argo_lua.parm')
    values = read_parameters(rover / 'newrovergazebo.params')
    values.update(selected)
    (run / 'scripts/modules').mkdir(parents=True, exist_ok=True)
    for name in ('system-start.lua', 'rover-controlallocation.lua', 'lidar_odometry.lua',
                 'modules/functions.lua', 'modules/pid.lua'):
        shutil.copy2(rover / 'scripts' / name, run / 'scripts' / name)
    (run / 'defaults.parm').write_text(''.join('%s %.10g\n' % item for item in values.items()))
    entries = '\n'.join('  {"%s", %.10g},' % item for item in selected.items())
    (run / 'scripts/00-ros-argo-config.lua').write_text('''-- Generated for SITL only.
local selected = {
%s
}
local function configure()
  if arming:is_armed() then return configure, 200 end
  if param:get('BATT_SOC_ALLOC') == nil then return configure, 200 end
  assert(param:set('BATT_SOC_ALLOC', 0))
  for _, item in ipairs(selected) do assert(param:set(item[1], item[2]), item[1]) end
  assert(param:set('BATT_SOC_WPYAW', 0))
  assert(param:set('BATT_SOC_ALLOC', 1))
  -- Publish the EKF's real origin after it becomes available. MAVROS
  -- guided_target subscribes to gp_origin; HOME_POSITION alone is not enough.
  -- Port 0 is the MAVProxy TCP link in this simulation.
  if gcs:set_message_interval(0, 49, 1000000) ~= 0 then
    return configure, 500
  end
  gcs:send_text(6, 'ROS Argo: Lua allocation enabled; simulation profile applied')
end
return configure, 1500
''' % entries)
    print('Lua/profile prepared in %s' % run)


if __name__ == '__main__':
    prepare(*sys.argv[1:])
