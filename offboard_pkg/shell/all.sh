#! /bin/bash

# Set FC into Hex+ mode
# ~/DartTracker/DartTracker 127.0.0.1 0.35 0.04 & PID0=$!
# sleep 5s
roslaunch tracker_pkg color_track.launch & PID0=$!
# roslaunch tracker_pkg tracker.launch & PID3=$!
# sleep 10s
roslaunch mavros px4.launch fcu_url:="/dev/ttyACM0:57600" & PID1=$!
sleep 10s
roslaunch offboard_pkg obs.launch & PID2=$!
sleep 5s

wait
# kill -9 PID0 PID1 PID2 PID3 PID4
kill -9 PID0 PID1 PID2
kill -9 `ps -e | grep DartTracker | awk 'NR==1{print $1}' `
exit