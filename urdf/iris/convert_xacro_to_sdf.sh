rosrun xacro xacro.py  iris_with_sensors.xacro > iris_with_sensors.urdf
gz sdf -p iris_with_sensors.urdf > iris_with_sensors.sdf
