# Command to launch the simulation
1
```bash
roslaunch unitree_gazebo robot_simulation.launch rname:=go1 wname:=office_cpr rviz:=false
```
2
```bash
rosrun unitree_guide junior_ctrl
```
3
```bash
roslaunch unitree_navigation navigation.launch rname:=go1 map_file:=/home/max/catkin_ws/src/ros_unitree/go1_path_finding/maps/office_cpr_all.yaml
```