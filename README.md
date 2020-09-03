# Victim-Localization
Victim Localization in Urban Search and Rescue

## Installing

Follow the steps below to install the simulation environment with all it's dependencies.

```
cd <catkin_ws>
wstool init src
wstool set -t src semantic_victim_localization https://github.com/kucars/semantic_victim_localization --git
wstool merge -t src https://raw.githubusercontent.com/kucars/semantic_victim_localization/master/victim_localization.rosinstall?token=AACHO4VKR6EP27G7VRLTW225NLC44/victim_localization.rosinstall
wstool update -t src
rosdep install -y -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO
catkin build
```


## Running
To run the main program, run the following commands in two separate terminals:

```
roslaunch victim_localization test.launch
roslaunch victim_localization NBV_test.launch
```
## Reference / Citation

If you find this code useful, please cite this in your work.

```
@article{ashour2020exploration,
  title={Exploration for Object Mapping Guided by Environmental Semantics using UAVs},
  author={Ashour, Reem and Taha, Tarek and Dias, Jorge Manuel Miranda and Seneviratne, Lakmal and Almoosa, Nawaf},
  journal={Remote Sensing},
  volume={12},
  number={5},
  pages={891},
  year={2020},
  publisher={Multidisciplinary Digital Publishing Institute}
}
```
