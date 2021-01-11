#! /bin/bash
# 2. start QGroundControl
# cd ~/Downloads
# ./QGroundControl.AppImage & PID1=$!
# sleep 10s

# 4. task scripts
# roslaunch mavros px4.launch fcu_url:="udp://:20101@192.168.0.1:20100" & PID1=$!
# sleep 2s
# roslaunch rflysim_ros_pkg cameras.launch & PID1=$!
# sleep 2s
roslaunch simulation rflysim_sphere.launch & PID1=$!
sleep 2s
roslaunch simulation sim_rate.launch & PID2=$!

# exit
wait
kill PID1 PID2
exit
