Brief: This ros package handles all received can messages, containing judging system data and gimbal/chassis board commands.
Maintainer: Yang Shaohui

sudo ip link set can0 type can bitrate 1000000
sudo ip link set up can0

sudo ip link set can1 type can bitrate 1000000
sudo ip link set up can1
