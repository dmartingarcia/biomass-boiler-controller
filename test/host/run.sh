#!/bin/sh
# Compiles and runs the host-side unit tests. No ESP32 or PlatformIO needed.
set -e
cd "$(dirname "$0")"
c++ -std=c++17 -Wall -Wextra -Wno-unused-parameter -I stub -I ../../include -o /tmp/boiler_test_core test_core.cpp
/tmp/boiler_test_core
