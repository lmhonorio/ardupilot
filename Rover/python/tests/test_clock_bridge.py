"""Clock decimation must preserve pauses and simulation resets."""
import importlib.util
from pathlib import Path
import unittest

spec=importlib.util.spec_from_file_location('clock_bridge',Path(__file__).resolve().parents[1]/'simulation/ros2/clock_bridge.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)

class ClockBridge(unittest.TestCase):
    def test_exact_samples_at_50hz(self):
        gate=module.ClockGate()
        samples=[ns for ns in range(0,100_000_001,4_000_000) if gate.accept(ns)]
        self.assertEqual(samples,list(range(0,100_000_001,20_000_000)))

    def test_pause_and_reset_do_not_invent_time(self):
        gate=module.ClockGate();self.assertTrue(gate.accept(10_000_000_000))
        for _ in range(10):self.assertFalse(gate.accept(10_000_000_000))
        self.assertTrue(gate.accept(0));self.assertFalse(gate.accept(0))
        self.assertTrue(gate.accept(20_000_000))

    def test_invalid_rate(self):
        for hz in (0,-1,1000):
            with self.assertRaises(ValueError):module.ClockGate(hz)

if __name__=='__main__':unittest.main()
