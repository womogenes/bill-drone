"""Run with: uv run --with websockets python src/fw/tests/test_ota_files.py."""

import contextlib
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

FIRMWARE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(FIRMWARE / "tools"))
sys.path.insert(0, str(FIRMWARE))
import ota_direct
from drivers import update


class Drone:
    def __init__(self, root):
        self.root = root
        self.requests = []
        self.restarts = 0

    def request(self, op, body=b"", **kwargs):
        self.requests.append(op)
        if op == b"I":
            return b"{}"
        with contextlib.chdir(self.root):
            return update.handle(op + body)

    def restart(self):
        self.restarts += 1


class FileTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        root = Path(temporary.name)
        self.local, self.remote = root / "local", root / "remote"
        for directory in (self.local, self.remote):
            (directory / "drivers").mkdir(parents=True)
            (directory / "drivers/imu.mpy").write_bytes(b"bytecode")
        self.drone = Drone(self.remote)

    def upload(self):
        with patch.object(ota_direct, "FIRMWARE", self.local), patch.object(ota_direct, "compile_driver", side_effect=lambda path: path):
            ota_direct.files(self.drone, ["drivers/imu.mpy"], True)

    def test_matching_bytecode_removes_old_source_and_restarts(self):
        source = self.remote / "drivers/imu.py"
        source.write_text("old source")
        self.upload()
        self.assertFalse(source.exists())
        self.assertNotIn(b"P", self.drone.requests)
        self.assertEqual(self.drone.restarts, 1)

    def test_unchanged_clean_install_does_not_restart(self):
        self.upload()
        self.assertEqual(self.drone.requests, [b"F"])
        self.assertEqual(self.drone.restarts, 0)

    def test_changed_bytecode_is_verified_then_source_removed(self):
        (self.remote / "drivers/imu.mpy").write_bytes(b"older bytecode")
        (self.remote / "drivers/imu.py").write_text("old source")
        self.upload()
        self.assertEqual((self.remote / "drivers/imu.mpy").read_bytes(), b"bytecode")
        self.assertFalse((self.remote / "drivers/imu.py").exists())
        self.assertEqual(self.drone.requests, [b"F", b"P", b"M", b"K", b"I"])

    def test_failed_upload_preserves_source(self):
        (self.remote / "drivers/imu.mpy").write_bytes(b"older bytecode")
        source = self.remote / "drivers/imu.py"
        source.write_text("old source")
        original = self.drone.request

        def fail_commit(op, body=b"", **kwargs):
            if op == b"M":
                raise RuntimeError("Injected verification failure")
            return original(op, body, **kwargs)

        self.drone.request = fail_commit
        with self.assertRaisesRegex(RuntimeError, "verification"):
            self.upload()
        self.assertTrue(source.exists())
        self.assertEqual(self.drone.restarts, 0)


if __name__ == "__main__":
    unittest.main()
