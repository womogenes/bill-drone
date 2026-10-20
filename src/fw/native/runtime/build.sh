#!/usr/bin/env bash
set -e

runtime_dir="$(cd "$(dirname "$0")" && pwd)"
native_dir="$(dirname "$runtime_dir")"
mpy_dir="$native_dir/.micropython"
idf_dir="${IDF_PATH:-$HOME/esp/esp-idf}"

if [ ! -d "$mpy_dir" ]; then
    git clone --depth 1 --branch v1.29.0 https://github.com/micropython/micropython.git "$mpy_dir"
fi

source "$idf_dir/export.sh"
set -u

# Keep unchanged firmware byte-identical across days.
export SOURCE_DATE_EPOCH="${SOURCE_DATE_EPOCH:-$(git -C "$mpy_dir" show -s --format=%ct HEAD)}"

if git -C "$mpy_dir" apply --check "$runtime_dir/micropython.patch" 2>/dev/null; then
    git -C "$mpy_dir" apply "$runtime_dir/micropython.patch"
else
    git -C "$mpy_dir" apply --reverse --check "$runtime_dir/micropython.patch"
fi

port_dir="$mpy_dir/ports/esp32"
case "${1:-factory}" in
    factory) board=board; build_dir="$native_dir/build-runtime" ;;
    ota) board=board_ota; build_dir="$native_dir/build-runtime-ota" ;;
    *) echo "Usage: $0 [factory|ota]" >&2; exit 2 ;;
esac
mkdir -p "$build_dir"
exec 9>"$build_dir/.build.lock"
flock 9
cmake_args=(
    "-DMICROPY_BOARD=$board"
    "-DMICROPY_BOARD_DIR=$runtime_dir/$board"
    "-DUSER_C_MODULES=$runtime_dir/micropython.cmake"
)

if [ ! -f "$build_dir/submodules/CMakeCache.txt" ]; then
    idf.py -C "$port_dir" -B "$build_dir/submodules" \
        "${cmake_args[@]}" -DUPDATE_SUBMODULES=1 reconfigure
fi
if [ -f "$build_dir/build.ninja" ]; then
    ninja -C "$build_dir"
else
    idf.py -C "$port_dir" -B "$build_dir" "${cmake_args[@]}" build
fi

if [ ! -f "$build_dir/firmware.bin" ] || \
    [ "$build_dir/micropython.bin" -nt "$build_dir/firmware.bin" ] || \
    [ "$build_dir/bootloader/bootloader.bin" -nt "$build_dir/firmware.bin" ] || \
    [ "$build_dir/partition_table/partition-table.bin" -nt "$build_dir/firmware.bin" ]; then
    python "$port_dir/makeimg.py" \
        "$build_dir/sdkconfig" \
        "$build_dir/bootloader/bootloader.bin" \
        "$build_dir/partition_table/partition-table.bin" \
        "$build_dir/micropython.bin" \
        "$build_dir/firmware.bin" \
        "$build_dir/micropython.uf2"
fi
