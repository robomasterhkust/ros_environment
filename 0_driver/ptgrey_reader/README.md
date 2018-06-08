# pointgrey

## Dependency

### usb 3.0 driver
解压 ./ptgrey_reader/install/usb/libusb-1.0.21.tar.bz2

```
cd ./ptgrey_reader/install/usb/libusb-1.0.21/

./configure

make

sudo make install

cd tests

make

./stress

```
看四个测试是否都通过

### amd64 ptgrey driver

解压 ./ptgrey_reader/install/amd64/flycapture2-2.11.3.121-amd64-pkg.tgz

```
cd flycapture2-2.11.3.121-amd64
```

(vim README 看需要装ubuntu 16.04 的dependency)

```
sudo apt-get install libraw1394-11 libgtkmm-2.4-dev                    \
        libglademm-2.4-dev libgtkglextmm-x11-1.2-dev libusb-1.0-0

sudo ./install_flycapture.sh   
```
(会有选项，打 y 加自己的用户名就行）

### (optional)dw
```
sudo apt-get install libdw-dev
```
（我们的ros_environment 里会删）

### （optional）系统设置，给权限
```
vim /etc/udev/rules.d/40-flir.rules 
```

把所有0664 给 0777

## catkin_make

## launch package

```
rosrun ptgrey_reader camera_list
```

看到         Serial number | 17591762

把那个 Serial number 改在launch/single.launch 里

开新tab，

```
roscore

sudo -s

source ~/ws/devel/setup.bash

roslaunch ptgrey_reader single.launch
```

### (optional) 增加usb buffer

```
sudo -S sh -c 'echo 2048 > /sys/module/usbcore/parameters/usbfs_memory_mb'
```

### 毛师傅的tricks

用双目的话也 一个一个起，时间戳打的准。

IMU用A3/N3的话，可以用同步，用A3/N3，用IMU触发摄像机，同步做的好。


## 调参

"shutter" 曝光时间
"exposure" 曝光值
"gain" 增益，default 0
