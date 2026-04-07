#!/usr/bin/env bash
set -euo pipefail

# Run unit tests using ESP-IDF's unit-test-app (QEMU)
# Usage: ./run_tests.sh

# If IDF_PATH is not set, try to infer a default location (you can override it by exporting IDF_PATH).
: "${IDF_PATH:=${HOME}/esp/v5.5.3/esp-idf}"

if [ ! -d "$IDF_PATH" ]; then
  echo "ERROR: IDF_PATH not found at '$IDF_PATH'"
  echo "Please set IDF_PATH to your ESP-IDF installation (e.g. export IDF_PATH=~/esp/esp-idf)"
  exit 1
fi

PROJECT_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

cat <<EOF
========================================
Running Unit Tests
Project: $PROJECT_PATH
IDF: $IDF_PATH
========================================
EOF

cd "$IDF_PATH/tools/unit-test-app"

echo "Building tests..."
python "$IDF_PATH/tools/idf.py" -D EXTRA_COMPONENT_DIRS="$PROJECT_PATH/components" build

echo
cat <<'EOF'
=======================================
Build successful!
=======================================
EOF

echo
python "$IDF_PATH/tools/idf.py" qemu monitor
