## precedure

- ssh victory@192.168.1.223
- roscore
- roslaunch rm_cv detector.launch
- /* debug 	得关 */
- roslaunch self_aiming rm_self_aiming_pid.launch
- y_kp -5.95
- y_kd -6.6
- z_kp 8.55
- center_x -0.02
- center_y -0.01
- k_y 1 (feedforward only)
- k_z 1 (feedforward only)
- Roslaunch usb_can usb_can.launch
