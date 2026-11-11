#!/usr/bin/env python3
"""Isolate Harmonic EEPROM/logs, retaining the user's parameter file and Lua."""
from pathlib import Path
import argparse
import shutil
import sys
ROVER=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROVER/'python/simulation'))
from prepare_ros_argo_lua import read_parameters


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--add-param-file',type=Path,default=ROVER/'nroverobst.param')
    parser.add_argument('--run-dir',type=Path,default=ROVER/'ros2/run/sitl')
    parser.add_argument('--sysid',type=int)
    a=parser.parse_args();values=read_parameters(a.add_param_file)
    if a.sysid is not None:
        if not 1<=a.sysid<=255:parser.error('sysid deve estar entre 1 e 255')
        values['SYSID_THISMAV']=a.sysid
    run=a.run_dir;run.mkdir(parents=True,exist_ok=True)
    shutil.copytree(ROVER/'scripts',run/'scripts',dirs_exist_ok=True)
    # Dynamic Lua parameters do not exist when SITL initially loads defaults.
    # Enable allocation only after the script creates its table, while disarmed.
    (run/'scripts/00-ros2-enable.lua').write_text('''local function enable()
  if arming:is_armed() then return enable, 500 end
  if param:get('BATT_SOC_ALLOC') == nil then return enable, 500 end
  assert(param:set('BATT_SOC_ALLOC', 1))
  if gcs:set_message_interval(0, 49, 1000000) ~= 0 then return enable, 500 end
  gcs:send_text(6, 'ROS2: alocacao Lua habilitada')
end
return enable, 1500
''')
    (run/'defaults.parm').write_text(''.join('%s %.10g\n'%item for item in values.items()))
    (run/'selected_parameter_file.txt').write_text(str(a.add_param_file.resolve())+'\n')
    print('Perfil:',a.add_param_file.resolve(),'| EEPROM isolada:',run)

if __name__=='__main__':main()
