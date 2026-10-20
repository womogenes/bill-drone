"""
Drivers run on the drone as bytecode: compiling them there at boot exhausts its heap.
"""

from pathlib import Path
import subprocess

FIRMWARE = Path(__file__).resolve().parents[1]


def compile_driver(name):
    """
    Return the path to upload for name, compiling drivers/*.py to .mpy.
    """
    if not (name.startswith("drivers/") and name.endswith(".py")) or name == "drivers/__init__.py":
        return name
    target = name[:-3] + ".mpy"
    subprocess.run(["uvx", "-q", "--from", "mpy-cross==1.29.0.post2", "mpy-cross", "-march=xtensawin",
                    str(FIRMWARE / name), "-o", str(FIRMWARE / target)], check=True)
    return target
