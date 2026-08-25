#!/bin/bash
# Regenerates MockBNO085_SPI_Library.{c,h} from include/BNO085_SPI_Library.h.
#
# Requires git and Ruby (>= 2.5). Use docker-compose.yml instead if you would
# rather not install Ruby.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" > /dev/null 2>&1 && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." > /dev/null 2>&1 && pwd)"
HEADER="$REPO_ROOT/include/BNO085_SPI_Library.h"

# Pinned so that regenerating produces byte-identical output over time. The CI
# staleness check depends on this.
CMOCK_VERSION="v2.7.0"

if [ ! -f "$HEADER" ]; then
  echo "error: cannot find $HEADER" >&2
  exit 1
fi

for tool in git ruby; do
  if ! command -v "$tool" > /dev/null 2>&1; then
    echo "error: $tool is required but not installed" >&2
    exit 1
  fi
done

WORK_DIR="$(mktemp -d)"
trap 'rm -rf "$WORK_DIR"' EXIT

# --recurse-submodules brings in the exact Unity revision this CMock release
# pins. The generator loads Unity's Ruby helpers, so it is required even though
# no Unity C source is shipped.
echo "Cloning CMock $CMOCK_VERSION..."
git -c advice.detachedHead=false clone --quiet --depth 1 --recurse-submodules \
  --shallow-submodules --branch "$CMOCK_VERSION" \
  https://github.com/ThrowTheSwitch/CMock.git "$WORK_DIR/cmock"

echo "Generating mock from $HEADER..."
(
  cd "$WORK_DIR/cmock"
  ruby lib/cmock.rb -o"$SCRIPT_DIR/cmock_config.yml" "$HEADER"
)

# CMock writes into a mocks/ subdirectory of its working directory.
GENERATED_DIR="$WORK_DIR/cmock/mocks"
for name in MockBNO085_SPI_Library.c MockBNO085_SPI_Library.h; do
  if [ ! -f "$GENERATED_DIR/$name" ]; then
    echo "error: CMock did not produce $name" >&2
    exit 1
  fi
  mv "$GENERATED_DIR/$name" "$SCRIPT_DIR/$name"
done

# The CMock runtime is shipped alongside the mock so that consumers need no
# Ruby. Unity is deliberately not vendored: PlatformIO supplies it to test
# builds, and a second copy would clash at link time.
for name in cmock.c cmock.h cmock_internals.h; do
  cp "$WORK_DIR/cmock/src/$name" "$SCRIPT_DIR/$name"
done

echo "Wrote MockBNO085_SPI_Library.{c,h} and the CMock $CMOCK_VERSION runtime."
