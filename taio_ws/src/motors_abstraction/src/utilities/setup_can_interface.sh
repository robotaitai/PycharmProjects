#!/bin/bash
sudo /sbin/ip link set can0 down
sudo busybox devmem 0x0c303000 32 0x0000C400
sudo busybox devmem 0x0c303008 32 0x0000C458
sudo busybox devmem 0x0c303010 32 0x0000C400
sudo busybox devmem 0x0c303018 32 0x0000C458
sudo modprobe can
sudo modprobe can_raw
sudo modprobe mttcan
#sudo /sbin/ip link set can0 type can restart-ms 100
sudo ifconfig can0 txqueuelen 1000
sudo /sbin/ip link set can0 up type can bitrate 1000000
