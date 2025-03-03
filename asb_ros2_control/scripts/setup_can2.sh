#!/bin/bash
sudo ip link set can2 down
sudo ip link set can2 type can bitrate 250000
sudo ip link set can2 txqueuelen 1000
sudo ip link set can2 up
