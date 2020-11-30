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
@article{goian2019victim,
  title={Victim Localization in USAR Scenario Exploiting Multi-Layer Mapping Structure},
  author={Goian, Abdulrahman and Ashour, Reem and Ahmad, Ubaid and Taha, Tarek and Almoosa, Nawaf and Seneviratne, Lakmal},
  journal={Remote Sensing},
  volume={11},
  number={22},
  pages={2704},
  year={2019},
  publisher={Multidisciplinary Digital Publishing Institute}
}
```
