import importlib.util
from pathlib import Path
import tempfile
import unittest

ROVER = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location(
    'prepare_ros_argo_lua', ROVER / 'python/simulation/prepare_ros_argo_lua.py')
prepare = importlib.util.module_from_spec(spec)
spec.loader.exec_module(prepare)


class ParameterPreparationTests(unittest.TestCase):
    def test_qgc_and_name_value_have_the_same_meaning(self):
        with tempfile.TemporaryDirectory() as folder:
            source = Path(folder) / 'parameters'
            source.write_text('# QGC\n1\t1\tSCR_ENABLE\t1\t2\n1 1 WP_SPEED 0.4 9\n')
            qgc = prepare.read_parameters(source)
            source.write_text('SCR_ENABLE 1\nWP_SPEED 0.4 # m/s\n')
            self.assertEqual(qgc, prepare.read_parameters(source))

    def test_invalid_rows_cannot_silently_turn_into_numeric_parameter_names(self):
        with tempfile.TemporaryDirectory() as folder:
            source = Path(folder) / 'parameters'
            for row in ('1 1\n', 'WP_SPEED nan\n', 'WP_SPEED 0.4 extra\n'):
                source.write_text(row)
                with self.subTest(row=row), self.assertRaises(ValueError):
                    prepare.read_parameters(source)

    def test_profile_deploys_lua_without_reusing_the_user_eeprom(self):
        with tempfile.TemporaryDirectory() as folder:
            run = Path(folder)
            prepare.prepare(ROVER, run)
            values = prepare.read_parameters(run / 'defaults.parm')
            self.assertEqual(values['SCR_ENABLE'], 1)
            self.assertEqual(values['SCR_USER5'], 2)
            self.assertEqual(values['ATC_STR_RAT_P'], 0.2)
            self.assertEqual(values['WP_SPEED'], 0.4)
            self.assertNotIn('1', values)
            self.assertFalse((run / 'eeprom.bin').exists())
            self.assertEqual((run / 'scripts/modules/functions.lua').read_bytes(),
                             (ROVER / 'scripts/modules/functions.lua').read_bytes())


if __name__ == '__main__':
    unittest.main()
