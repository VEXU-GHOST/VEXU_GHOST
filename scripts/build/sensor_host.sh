#!/bin/bash

# Builds and flashes the ghost_sensor_host RP2040 polling firmware.
#
#   Submodules (lib/FreeRTOS-Kernel, lib/Library-VectorQuaternionMatrix):
#       scripts/setup_submodules.sh
#   Toolchain + Pico SDK + picotool:
#       scripts/update_dependencies.sh
#
# Pass -b / --no-flash to build only (skip flashing).

# Help printout
if [ "$1" == "-h" ] || [ "$1" == "--help" ]; then
    echo "Builds (and flashes) the ghost_sensor_host RP2040 polling firmware."
    echo "Specify -b / --no-flash to build without flashing."
    exit 0
fi

FLASH=1
if [ "$1" == "-b" ] || [ "$1" == "--no-flash" ]; then
    FLASH=0
fi

# Verify repo path is set
if [ -z "${VEXU_HOME}" ]; then
    echo "Failure: repository path variable VEXU_HOME is unset."
    exit -1
fi

FW_DIR="$VEXU_HOME/02_V5/ghost_sensor_host/host_firmware/polling_firmware"
BUILD_DIR="$FW_DIR/build"
UF2="$BUILD_DIR/polling_firmware.uf2"

# ---- Locate the Pico SDK --------------------------------------------------
# Honour an explicit PICO_SDK_PATH, otherwise fall back to the copy that
# update_dependencies.sh clones into 09_External, then the VS Code extension's.
if [ -z "$PICO_SDK_PATH" ]; then
    if [ -d "$VEXU_HOME/09_External/pico-sdk" ]; then
        export PICO_SDK_PATH="$VEXU_HOME/09_External/pico-sdk"
    elif [ -d "$HOME/.pico-sdk/sdk" ]; then
        export PICO_SDK_PATH="$(ls -d "$HOME"/.pico-sdk/sdk/*/ 2>/dev/null | sort -V | tail -1)"
    fi
fi
if [ -z "$PICO_SDK_PATH" ] || [ ! -d "$PICO_SDK_PATH" ]; then
    echo "ERROR: Pico SDK not found. Run scripts/update_dependencies.sh (or set PICO_SDK_PATH)."
    exit -1
fi

# ---- Build ----------------------------------------------------------------
echo
echo -------------------------------------------------------
echo ---------- Building Sensor Host Firmware ---------------
echo -------------------------------------------------------
echo "Using PICO_SDK_PATH=$PICO_SDK_PATH"

cmake -S "$FW_DIR" -B "$BUILD_DIR" -DPICO_SDK_PATH="$PICO_SDK_PATH" || exit -1
cmake --build "$BUILD_DIR" -j"$(nproc)" || exit -1

if [ ! -f "$UF2" ]; then
    echo "ERROR: build did not produce $UF2"
    exit -1
fi

if [ "$FLASH" -eq 0 ]; then
    echo "Build complete: $UF2 (flashing skipped)."
    exit 0
fi

# ---- Flash ----------------------------------------------------------------
echo
echo -------------------------------------------------------
echo ---------- Flashing Sensor Host Firmware ---------------
echo -------------------------------------------------------

# Preferred: picotool reboots the running board into BOOTSEL (-f) and loads it.
if command -v picotool >/dev/null 2>&1; then
    picotool load -x -f "$UF2" && { echo "Flashed via picotool."; exit 0; }
    echo "picotool could not reach the board; trying BOOTSEL mass-storage copy..."
fi

# Fallback: copy the UF2 onto a board already mounted in BOOTSEL mode.
MOUNT="$(findmnt -rn -o TARGET -S LABEL=RPI-RP2 2>/dev/null | head -1)"
if [ -z "$MOUNT" ]; then
    for d in "/media/$USER/RPI-RP2" "/run/media/$USER/RPI-RP2" "/media/$USER/RP2350"; do
        [ -d "$d" ] && MOUNT="$d" && break
    done
fi
if [ -n "$MOUNT" ]; then
    cp "$UF2" "$MOUNT/" && sync && { echo "Flashed by copying to $MOUNT."; exit 0; }
fi

echo "WARNING: no Pico found to flash."
echo "  - Install picotool (scripts/update_dependencies.sh) for one-step flashing, or"
echo "  - hold BOOTSEL while plugging in the Pico and re-run, or copy manually:"
echo "      $UF2"
exit -1
