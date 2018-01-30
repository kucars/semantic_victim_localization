rosrun xacro xacro.py  iris_with_sensors_test.xacro > iris_with_sensors_test.urdf
gz sdf -p iris_with_sensors_test.urdf > iris_with_sensors_test.sdf
