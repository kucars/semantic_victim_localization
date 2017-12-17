# Victim-Localization
Victim Localization in Urban Search and Rescue

## Installing

Follow the steps below to install the simulation environment with all it's dependencies.

```
cd <catkin_ws>
wstool init src
wstool set -t src Victim-Localization https://github.com/AbdulrahmanGoian/Victim-Localization --git
wstool merge -t src https://raw.githubusercontent.com/https://github.com/AbdulrahmanGoian/Victim-Localization/master/victim_localization.rosinstall
wstool update -t src
rosdep install -y -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO
catkin build
```


## Running
To run the main program, run the following commands in two separate terminals:

```
roslaunch Victim-Localization test.launch
roslaunch Victim-Localization NBV_test.launch
```
