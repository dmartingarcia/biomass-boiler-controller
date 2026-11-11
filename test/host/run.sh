#!/bin/sh
# Compiles and runs the host-side unit tests. No ESP32 needed.
# The JSON tests use ArduinoJson from .pio/libdeps (present after one `pio run`); skipped otherwise.
set -e
cd "$(dirname "$0")"
OUT="${TMPDIR:-/tmp}"
c++ -std=c++17 -Wall -Wextra -Wno-unused-parameter -I stub -I ../../include -o "$OUT/boiler_test_core" test_core.cpp
"$OUT/boiler_test_core"

AJ=$(ls -d ../../.pio/libdeps/*/ArduinoJson/src 2>/dev/null | head -n 1 || true)
if [ -n "$AJ" ]; then
    c++ -std=c++17 -Wall -Wextra -Wno-unused-parameter -I stub -I ../../include -I "$AJ" -o "$OUT/boiler_test_rules" test_rules.cpp
else
    c++ -std=c++17 -Wall -Wextra -Wno-unused-parameter -DNO_JSON_TESTS -I stub -I ../../include -o "$OUT/boiler_test_rules" test_rules.cpp
fi
"$OUT/boiler_test_rules"
