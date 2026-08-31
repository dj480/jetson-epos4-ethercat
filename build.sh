#!/bin/bash
# Compile c_src/motor.c against SOEM into lib/libmotor.so.
#
# SOEM headers are split across include/, osal/, osal/linux/, and
# oshw/linux/ (osal.h and oshw.h each pull in an OS-specific companion header
# with no subpath, so all four -I paths are required). libsoem.a is the
# static SOEM build; -fPIC is required on both sides since a static archive
# gets linked directly into our shared library.
set -euo pipefail

SOEM_ROOT="/home/danielbetts/SOEM"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

gcc -shared -fPIC -o "$SCRIPT_DIR/lib/libmotor.so" "$SCRIPT_DIR/c_src/motor.c" \
  -I "$SOEM_ROOT/include" \
  -I "$SOEM_ROOT/osal" \
  -I "$SOEM_ROOT/osal/linux" \
  -I "$SOEM_ROOT/oshw/linux" \
  "$SOEM_ROOT/build/libsoem.a" \
  -lpthread

echo "Built $SCRIPT_DIR/lib/libmotor.so"
