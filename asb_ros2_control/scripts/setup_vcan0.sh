#!/bin/bash
sleep 5
modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
