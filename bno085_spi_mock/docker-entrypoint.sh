#!/bin/bash
set -e

# Debug information
echo "Checking CMock setup..."
ls -la /cmock
echo "Checking vendor directory..."
ls -la /cmock/vendor
echo "Checking Unity directory..."
ls -la /cmock/vendor/unity || echo "Unity directory not found!"

# Run CMock
echo "Running CMock..."
cd /app
ruby /cmock/lib/cmock.rb -ocmock_config.yml BNO085_SPI_Library.h

# List generated files
echo "Generated files:"
ls -la mocks/

# No need to copy files since they're already in the mounted volume
echo "Done! Check the mocks/ directory for generated files."
