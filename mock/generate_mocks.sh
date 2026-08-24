#!/bin/bash
set -euo pipefail

# Script to generate CMock mocks for BNO085_SPI_Library.h
# Requires: Ruby >= 2.5, git

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
MOCK_DIR="$SCRIPT_DIR"
CMOCK_VERSION="v2.7.0"

echo "Generating mocks for BNO085_SPI_Library.h..."
echo "Repository root: $REPO_ROOT"
echo "Mock directory: $MOCK_DIR"
echo "CMock version: $CMOCK_VERSION"

# Create a temporary directory for CMock
TEMP_DIR=$(mktemp -d)
trap 'rm -rf "$TEMP_DIR"' EXIT

cd "$TEMP_DIR"

# Clone CMock at the pinned version
echo "Cloning CMock $CMOCK_VERSION..."
git clone --depth 1 --branch "$CMOCK_VERSION" https://github.com/ThrowTheSwitch/CMock.git
cd CMock

# Install CMock dependencies (Unity)
echo "Installing Unity..."
git clone --depth 1 https://github.com/ThrowTheSwitch/Unity.git vendor/unity

# Generate the mock
echo "Generating mock files..."
ruby lib/cmock.rb -o"$MOCK_DIR/cmock_config.yml" "$REPO_ROOT/include/BNO085_SPI_Library.h"

# Find and move generated files to the mock directory
echo "Moving generated files..."
# CMock generates files in a mocks/ subdirectory by default
if [ -f "mocks/MockBNO085_SPI_Library.c" ]; then
    mv mocks/MockBNO085_SPI_Library.c "$MOCK_DIR/"
    mv mocks/MockBNO085_SPI_Library.h "$MOCK_DIR/"
elif [ -f "MockBNO085_SPI_Library.c" ]; then
    mv MockBNO085_SPI_Library.c "$MOCK_DIR/"
    mv MockBNO085_SPI_Library.h "$MOCK_DIR/"
elif [ -f "../MockBNO085_SPI_Library.c" ]; then
    mv ../MockBNO085_SPI_Library.c "$MOCK_DIR/"
    mv ../MockBNO085_SPI_Library.h "$MOCK_DIR/"
else
    echo "Error: Generated mock files not found!"
    find . -name "MockBNO085_SPI_Library.*"
    exit 1
fi

# Copy CMock runtime files if they don't exist
if [ ! -f "$MOCK_DIR/cmock.c" ]; then
    echo "Copying CMock runtime files..."
    cp src/cmock.c "$MOCK_DIR/"
    cp src/cmock.h "$MOCK_DIR/"
    cp src/cmock_internals.h "$MOCK_DIR/"
fi

# Copy Unity runtime files if they don't exist
if [ ! -f "$MOCK_DIR/unity.c" ]; then
    echo "Copying Unity runtime files..."
    cp vendor/unity/src/unity.c "$MOCK_DIR/"
    cp vendor/unity/src/unity.h "$MOCK_DIR/"
    cp vendor/unity/src/unity_internals.h "$MOCK_DIR/"
fi

echo "Mock generation complete!"
echo "Generated files:"
echo "  - MockBNO085_SPI_Library.c"
echo "  - MockBNO085_SPI_Library.h"
