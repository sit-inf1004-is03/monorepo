#!/bin/sh
# Build and run the PC unit tests for the IMU subsystem's pure modules.
# Usage (from any directory):  sh host_test/run_host_tests.sh
# Needs any C99 compiler on the PATH as "gcc" (or set CC=...).

set -e

HERE=$(cd "$(dirname "$0")" && pwd)
SRC="$HERE/../app_program"
OUT="$HERE/build"
CC=${CC:-gcc}

mkdir -p "$OUT"

$CC -std=c99 -O2 -Wall -Wextra -Wshadow -Wdouble-promotion -Wundef \
    -DIMU_HOST_TEST \
    "$SRC/common/bitpack.c" \
    "$SRC/subsystem4/imu_math.c" \
    "$SRC/subsystem4/imu_filter.c" \
    "$SRC/subsystem4/imu_odom.c" \
    "$SRC/subsystem4/imu_motion.c" \
    "$SRC/subsystem4/imu_terrain.c" \
    "$SRC/subsystem4/imu_impact.c" \
    "$SRC/subsystem4/imu_pack.c" \
    "$SRC/subsystem4/imu_monitor.c" \
    "$SRC/subsystem4/imu_sim.c" \
    "$SRC/subsystem4/imu.c" \
    "$HERE/test_imu_host.c" \
    -lm -o "$OUT/test_imu_host"

"$OUT/test_imu_host"
