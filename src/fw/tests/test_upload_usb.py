"""Run with: uv run --offline --with mpremote python src/fw/tests/test_upload_usb.py."""

import contextlib
import errno
import io
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import upload_usb


class Board:
    """Execute the uploader's remote Python against a temporary filesystem."""

    def __init__(self, root):
        self.root = root
        self.namespace = {}
        self.writes = []
        self.corrupt = False

    def exec(self, code):
        output = io.StringIO()
        with contextlib.chdir(self.root), contextlib.redirect_stdout(output):
            exec(code, self.namespace)
        return output.getvalue().encode()

    def fs_writefile(self, path, data, chunk_size):
        self.writes.append(path)
        (self.root / path).write_bytes(b"corrupt" if self.corrupt else data)


class UploadTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        root = Path(temporary.name)
        self.local = root / "local"
        self.remote = root / "remote"
        for directory in (self.local, self.remote):
            (directory / "drivers").mkdir(parents=True)
        self.source = self.local / "drivers/imu.mpy"
        self.source.write_bytes(b"new bytecode")
        self.shadow = self.remote / "drivers/imu.py"
        self.shadow.write_text("old source")
        self.bytecode = self.remote / "drivers/imu.mpy"
        self.bytecode.write_bytes(self.source.read_bytes())
        self.board = Board(self.remote)
        patcher = patch.object(upload_usb, "FIRMWARE", self.local)
        patcher.start()
        self.addCleanup(patcher.stop)

    def test_matching_bytecode_still_removes_shadow(self):
        self.assertEqual(upload_usb.upload(self.board, [self.source]), (0, 0))
        self.assertFalse(self.shadow.exists())
        self.assertEqual(self.board.writes, [])

    def test_changed_bytecode_is_verified_before_shadow_removed(self):
        self.bytecode.write_bytes(b"old bytecode")
        changed, transferred = upload_usb.upload(self.board, [self.source])
        self.assertEqual((changed, transferred), (1, len(self.source.read_bytes())))
        self.assertEqual(self.bytecode.read_bytes(), self.source.read_bytes())
        self.assertFalse(self.shadow.exists())

    def test_missing_shadow_is_harmless(self):
        self.shadow.unlink()
        self.assertEqual(upload_usb.upload(self.board, [self.source]), (0, 0))

    def test_remove_failure_is_reported(self):
        with patch("os.remove", side_effect=OSError(errno.EACCES, "Permission denied")):
            with self.assertRaises(PermissionError):
                upload_usb.upload(self.board, [self.source])
        self.assertTrue(self.shadow.exists())

    def test_failed_verification_preserves_existing_code(self):
        self.bytecode.write_bytes(b"old bytecode")
        self.board.corrupt = True
        with self.assertRaisesRegex(RuntimeError, "verification failed"):
            upload_usb.upload(self.board, [self.source])
        self.assertEqual(self.bytecode.read_bytes(), b"old bytecode")
        self.assertTrue(self.shadow.exists())

    def test_python_source_is_retained(self):
        source = self.local / "main.py"
        source.write_text("print(1)")
        upload_usb.upload(self.board, [source])
        self.assertEqual((self.remote / "main.py").read_text(), "print(1)")

    def test_multiple_serial_devices_require_explicit_selection(self):
        ports = [SimpleNamespace(device=name, vid=0x10C4) for name in ("/dev/ttyUSB0", "/dev/ttyUSB1")]
        with patch.object(upload_usb.list_ports, "comports", return_value=ports):
            with self.assertRaisesRegex(SystemExit, "Choose --port explicitly"):
                upload_usb.select_port("auto")
            self.assertEqual(upload_usb.select_port("/dev/ttyUSB1"), "/dev/ttyUSB1")


if __name__ == "__main__":
    unittest.main()
