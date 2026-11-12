"""Profile boundaries: physical hardware must not inherit SITL-only behavior."""
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch
import yaml

ROVER=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROVER/'python'))
spec=importlib.util.spec_from_file_location('companion_stack',ROVER/'companion/stack.py')
stack=importlib.util.module_from_spec(spec);spec.loader.exec_module(stack)
from common.hardware import calibration
from mission_manager.manager import Manager


class Profiles(unittest.TestCase):
    def config(self,name):return yaml.safe_load((ROVER/'companion/config'/name).read_text())
    def test_simulation_has_one_manager_and_only_udp_master(self):
        plan=dict(stack.commands(self.config('simulation.yaml'),Path('/data/test')))
        self.assertIn('--simulate-charge',plan['manager'])
        self.assertEqual(plan['manager'][plan['manager'].index('--vehicle')+1],'sitl')
        self.assertIn('udpin:0.0.0.0:14600',plan['mavproxy'])
        self.assertNotIn('livox',plan)
        source='/rover_argo_N1/Instance1/livox/lidar'
        target=source+'_onboard'
        self.assertIn('source:='+source,plan['sensor_ingress'])
        self.assertIn(target,plan['fastlio'])
        self.assertIn(target,plan['manager'])
        self.assertIn('--external-simulation-adapter',plan['fastlio'])
        self.assertIn('source:='+target,plan['simulation_adapter'])
        self.assertNotIn(source,plan['fastlio'])
    def test_real_refuses_uncommissioned_configuration(self):
        with self.assertRaisesRegex(ValueError,'calibrado'):stack.commands(self.config('real.yaml'),Path('/data/test'))
    def test_real_commands_never_enable_artificial_charging(self):
        c=self.config('real.yaml');c['hardware_configured']=True
        plan=dict(stack.commands(c,Path('/data/test')))
        self.assertIn('--no-simulate-charge',plan['manager'])
        self.assertNotIn('--simulate-charge',plan['manager'])
        self.assertIn('/data/real/recovery.json',plan['manager'])
        self.assertNotIn('/data/simulation/recovery.json',plan['manager'])
        self.assertIn('--sensor-profile',plan['fastlio']);self.assertIn('real',plan['fastlio'])
        self.assertIn('livox',plan)
        self.assertNotIn('simulation_adapter',plan)
        self.assertNotIn('--external-simulation-adapter',plan['fastlio'])
    def test_simulation_rejects_serial_master(self):
        c=self.config('simulation.yaml');c['mavproxy_master']='/dev/ttyACM0'
        with self.assertRaisesRegex(ValueError,'UDP'):stack.commands(c,Path('/data/test'))
    def test_hardware_template_not_accepted_as_calibration(self):
        with self.assertRaisesRegex(ValueError,'validada'):calibration(ROVER/'companion/config/hardware.example.yaml')
    def test_bad_rotation_and_nonfinite_translation_rejected(self):
        c=self.config('hardware.example.yaml');c.update(validated=True,imu_axes_aligned_to_base=True)
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'hardware.yaml';c['base_to_imu'][0]=float('nan');p.write_text(yaml.safe_dump(c))
            with self.assertRaisesRegex(ValueError,'Translacao'):calibration(p)
            c['base_to_imu'][0]=0.;c['lidar_to_imu_rotation'][0]=2.;p.write_text(yaml.safe_dump(c))
            with self.assertRaisesRegex(ValueError,'Rotacao'):calibration(p)
    def test_real_telemetry_does_not_require_simstate(self):
        m=Manager.__new__(Manager);m.args=NS(vehicle='real')
        m.link=NS(last_seen={k:time.monotonic() for k in ('HEARTBEAT','GLOBAL_POSITION_INT','ATTITUDE','VFR_HUD')})
        self.assertTrue(m.fresh());m.args.vehicle='sitl';self.assertFalse(m.fresh())
        m.link.last_seen['SIMSTATE']=time.monotonic();self.assertTrue(m.fresh())
    def test_real_still_rejects_missing_attitude(self):
        m=Manager.__new__(Manager);m.args=NS(vehicle='real');m.link=NS(last_seen={})
        self.assertFalse(m.fresh())

if __name__=='__main__':unittest.main()
