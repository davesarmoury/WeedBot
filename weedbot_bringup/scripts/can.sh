#!/bin/bash
sudo modprobe mttcan
sudo ip link set can0 up type can bitrate 500000
