#!/bin/bash

sudo /sbin/ip link set can0 down
sudo /sbin/ip link set can0 up type can bitrate 1000000 restart-ms 100
