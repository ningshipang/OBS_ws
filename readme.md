## Usage

### code explanation
```
obs.py including utils_obs.py:for velocity command
obs_rate.py including utils_angule.py:for attitude command, namely angular velocity and thrust
python PATH_TO_LOG variables ...
# for example
# python plot_data.py ../simulation/log/20200923_104623_sim.log rpos_est_body_raw depth rpos_est_body
```

### Simulation with rflysim about velocity controlling
In `settings.json`, "MODE": "Simulation".
```
# first terminal:mavros initiation
roslaunch mavros px4.launch fcu_url:="udp://:20101@192.168.0.1:20100"
# second terminal:read image from rflysim
roslaunch rflysim_ros_pkg cameras.launch
# third terminal:read image from rflysim
roscd simulation/shell/
./rfly_hitl.sh | tee -a `roscd simulation/log/ && pwd`/`date +%Y%m%d_%H%M%S_sim.log`
```

### Simulation with rflysim about attitude controlling
In `settings.json`, "MODE": "Simulation".
```
# first terminal:mavros initiation
roslaunch mavros px4.launch fcu_url:="udp://:20101@192.168.0.1:20100"
# second terminal:read image from rflysim
roslaunch rflysim_ros_pkg cameras.launch
# third terminal:read image from rflysim
roscd simulation/shell/
./gazebo-hitl.sh | tee -a `roscd simulation/log/ && pwd`/`date +%Y%m%d_%H%M%S_sim.log`
```

### Real flight
In `settings.json`, "MODE": "RealFlight".
```
roscd offboard_pkg/shell/
./all.sh | tee -a `roscd simulation/log/ && pwd`/`date +%Y%m%d_%H%M%S_fly.log`
```

### Analyse log
```
cd PATH_TO_analyse
python PATH_TO_LOG variables ...
#plot_data.py: varities with time
#plot_route.py:variables with x-y
#plot_multiple_file.py: plot all
# for example
# python plot_data.py ../simulation/log/20200923_104623_sim.log rpos_est_body_raw depth rpos_est_body
```