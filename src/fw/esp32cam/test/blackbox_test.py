"""
Round trip of the black-box format: builds test/blackbox_test.cpp with the camera's src/blackbox.h, writes a file, and
checks that blackbox.py decodes every queued frame (record, reduced frame, tag crop, full frame) exactly and counts the
drops. Run from src/fw: uv run esp32cam/test/blackbox_test.py
"""

import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

here = Path(__file__).resolve().parent
sys.path.insert(0, str(here.parent))
import blackbox  # noqa: E402

with tempfile.TemporaryDirectory() as work:
    work = Path(work)
    program = work / "blackbox_test"
    subprocess.run(["c++", "-std=c++17", "-O1", "-Wall", "-Wextra", "-Werror", "-fsanitize=address,undefined", str(here / "blackbox_test.cpp"), "-o", str(program)], check=True)
    output = subprocess.run([str(program), str(work / "test.bin")], check=True, capture_output=True, text=True).stdout.split("\n")
    offered = [tuple(int(v) for v in line.split()) for line in output if line and not line.startswith("dropped")]
    dropped = int(next(line for line in output if line.startswith("dropped")).split()[1])
    queued = [o for o in offered if o[1]]
    data = (work / "test.bin").read_bytes()
    frames = [e for e in blackbox.entries(data) if "record" in e]
    assert len(frames) == len(queued), (len(frames), len(queued))
    assert dropped > 0 and frames[-1]["dropped"] <= dropped
    x, y = np.meshgrid(np.arange(320), np.arange(240))
    for (sequence, _, left, top, full), e in zip(queued, frames):
        assert e["record"]["sequence"] == sequence
        assert e["record"]["age_us"] == 36000
        image = ((x * 3 + y * 7 + sequence) & 0xFF).astype(np.uint8)
        assert np.array_equal(e["reduced"].ravel(), ((np.arange(80 * 60) + sequence) & 0xFF).astype(np.uint8))
        if left < 0:
            assert e["crop"] is None
        else:
            (cl, ct), crop = e["crop"]
            assert (cl, ct) == (left, top) and crop.shape == (96, 96)
            assert np.array_equal(crop, image[top:top + 96, left:left + 96]), sequence
        assert (e["full"] is not None) == bool(full)
        if full:
            assert np.array_equal(e["full"], image)
    summary = blackbox.summarize(work / "test.bin")
    assert summary["dropped_by_black_box"] == frames[-1]["dropped"]
    assert summary["sequence_gaps"] == len(offered) - len(queued) - (offered[-1][0] - queued[-1][0])
    print(f"black box round trip: {len(frames)} frames decoded exactly, {dropped} dropped while the writer stalled")
