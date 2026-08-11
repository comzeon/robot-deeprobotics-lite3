#!/bin/bash
# Launch rbnx boot detached on Thor (survives ssh disconnect).
set -a; source ~/Desktop/robonix/.env; set +a
export PATH=$PATH:/home/nvidia/.cargo/bin:/home/nvidia/.local/bin
export ROBONIX_SOURCE_PATH=/home/nvidia/Desktop/robonix
cd ~/Desktop/robot-deeprobotics-lite3
nohup rbnx boot -f robonix_manifest.yaml --no-update-check > /tmp/rbnx-boot.log 2>&1 &
echo "boot pid=$!"
