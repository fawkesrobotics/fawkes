#!/bin/bash

# Build script, will (usually) be executed in Docker container

# Source standard environment, may setup ccache if available
source /etc/profile

# Error out on any error in the script, pipes etc.
set -euo pipefail

# Ensure no old build artifacts remain
make -j$(nproc) clean
rm -rf bin/ lib/ plugins/

# Build software
make -j$(nproc)

