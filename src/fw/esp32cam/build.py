"""
Keep AprilTag's pose solver in flash so Wi-Fi and camera ISRs fit in ESP32 IRAM.
The detector's hot paths retain their upstream placement. No vendor files change.
"""

from pathlib import Path

Import("env")

header = Path(env.subst("$BUILD_DIR")) / "apriltag_flash.h"
header.parent.mkdir(parents=True, exist_ok=True)
header.write_text('#include "esp_attr.h"\n#undef IRAM_ATTR\n#define IRAM_ATTR\n')


def pose_in_flash(build_env, node):
    return build_env.Object(
        node, CCFLAGS=build_env["CCFLAGS"] + ["-include", str(header)]
    )


env.AddBuildMiddleware(pose_in_flash, "*/apriltag_pose.c")
