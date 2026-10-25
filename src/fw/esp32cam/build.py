"""
Build the AprilTag detector at -O2 instead of the framework's -Os: it is the camera's hot path. Its cold files (the unused pose solver, matrix and geometry helpers, file formats) go to flash so Wi-Fi and camera interrupt code still fit in IRAM. No vendor files change.
"""

from pathlib import Path

Import("env")

header = Path(env.subst("$BUILD_DIR")) / "apriltag_flash.h"
header.parent.mkdir(parents=True, exist_ok=True)
header.write_text('#include "esp_attr.h"\n#undef IRAM_ATTR\n#define IRAM_ATTR\n')
# Float arguments call the float math functions, and the detector runs its stages as one task each.
single = Path(env.subst("$BUILD_DIR")) / "apriltag_single.h"
single.write_text(
    "#include <math.h>\n"
    + "".join(f"#define {f}(x) _Generic((x), float: {f}f, default: {f})(x)\n" for f in ["sqrt", "fabs", "exp", "log", "floor", "ceil", "round", "cos", "sin", "tan", "acos", "asin", "atan"])
    + "".join(f"#define {f}(y, x) _Generic((y), float: {f}f, default: {f})(y, x)\n" for f in ["atan2", "pow", "fmod"])
    # Single-threaded, splitting work into ten tasks per thread only multiplies per-task setup: each clustering task zeroes a 32 KB entry pool in PSRAM.
    + '#include "apriltag.h"\n#undef APRILTAG_TASKS_PER_THREAD_TARGET\n#define APRILTAG_TASKS_PER_THREAD_TARGET 1\n'
)


COLD = {"apriltag_pose.c", "matd.c", "homography.c", "svd22.c", "g2d.c", "workerpool.c", "pthreads_cross.c",
        "image_u8x3.c", "image_u8x4.c", "pam.c", "pnm.c", "pjpeg.c", "pjpeg-idct.c", "getopt.c", "string_util.c",
        "time_util.c"}


def optimize(build_env, node):
    # The library was converted to float, but its unsuffixed constants and math calls still promote to double, which the ESP32 only emulates in software. Make constants float and math calls type-generic.
    flags = [flag for flag in build_env["CCFLAGS"] if flag != "-Os"] + ["-O2", "-fsingle-precision-constant"]
    # The flash header goes first: the single-precision header includes apriltag.h, whose IRAM_ATTR declarations would otherwise put the cold definitions back in IRAM.
    if node.name in COLD:
        flags += ["-include", str(header)]
    if node.name.endswith(".c"):
        flags += ["-include", str(single)]
    return build_env.Object(node, CCFLAGS=flags)


env.AddBuildMiddleware(optimize, "*Apriltag library for Arduino ESP32*")


def optimize_own(build_env, node):
    return build_env.Object(node, CCFLAGS=[flag for flag in build_env["CCFLAGS"] if flag != "-Os"] + ["-O2"])


# Our per-frame tracking and flow loops too.
env.AddBuildMiddleware(optimize_own, "*/src/main.cpp")
