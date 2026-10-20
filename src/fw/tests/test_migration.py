"""Run with: python src/fw/tests/test_migration.py."""

import importlib.util
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

FIRMWARE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(FIRMWARE))


class MigrationTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.store = SimpleNamespace(migrated=Mock(return_value=False), migrate=Mock())
        spec = importlib.util.spec_from_file_location("migration_test_target", FIRMWARE / "drivers/migrate.py")
        self.module = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, {"settings": self.store}):
            spec.loader.exec_module(self.module)
        root = Path(temporary.name)
        self.tuning, self.calibration = root / "tuning.json", root / "calibration.json"
        self.module.TUNING_FILE, self.module.CALIBRATION_FILE = str(self.tuning), str(self.calibration)
        self.tuning.write_text(json.dumps({"pitch_p": 25, "servo_offsets": {"left": 8, "right": -2}}))
        self.calibration.write_text(json.dumps({
            "version": 1, "sensor_to_body": list(self.module.SENSOR_TO_BODY),
            "gyro_bias_rad_s": [0.01, 0.02, 0.03], "pose_zero_deg": [0.1, 0.2],
        }))

    def test_first_import_retains_sources_until_nvs_is_restored(self):
        self.module.run()
        self.assertTrue(self.tuning.exists() and self.calibration.exists())
        values = self.store.migrate.call_args.args[0]
        self.assertEqual(values["servo_offset_left"], 8)
        self.assertEqual(values["calibrated"], 1)
        self.assertEqual(values["gyro_bias_z"], 0.03)

    def test_restored_migration_removes_stale_sources(self):
        self.store.migrated.return_value = True
        self.module.run()
        self.assertFalse(self.tuning.exists() or self.calibration.exists())
        self.store.migrate.assert_not_called()
        self.module.run()  # Missing legacy files are expected on later boots.

    def test_lost_store_cannot_resurrect_old_calibration(self):
        self.store.migrated.return_value = True
        self.module.run()
        self.store.migrated.return_value = False
        self.module.run()
        self.store.migrate.assert_called_once_with({})

    def test_removal_failure_is_reported(self):
        self.store.migrated.return_value = True
        with patch.object(self.module.os, "remove", side_effect=PermissionError(13, "denied")):
            with self.assertRaises(PermissionError):
                self.module.run()
        self.store.migrate.assert_not_called()


if __name__ == "__main__":
    unittest.main()
