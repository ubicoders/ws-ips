#!/usr/bin/env bash
# The standard packages for the Rust opencv crate (ips_detect, ips_calib).
set -e
sudo apt-get update
sudo apt-get install -y libopencv-dev llvm-18 libclang-18-dev
