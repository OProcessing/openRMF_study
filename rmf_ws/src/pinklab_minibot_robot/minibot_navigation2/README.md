# minibot_navigation2

## Map Building

### For Gazebo simulation
```shell
$ ros2 launch minibot_bringup bringup_robot_gazebo.launch.xml world_name:=simple_building.world
```
```shell
$ ros2 launch minibot_navigation2 map_building.launch.xml use_sim_time:=true
```
```shell
$ rviz2 -d `ros2 pkg prefix minibot_navigation2`/share/minibot_navigation2/rviz/map_building.rviz
```
```shell
$ ros2 run teleop_twist_keyboard teleop_twist_keyboard
```


### For Navigation (Gazebo)
```shell
$ ros2 launch minibot_bringup bringup_robot_gazebo.launch.xml world_name:=simple_building.world
```
```shell
$ ros2 launch minibot_navigation2 bringup_launch.py use_sim_time:=true map:=`ros2 pkg prefix minibot_navigation2`/share/minibot_navigation2/maps/simple_building.yaml
```
```shell
$ rviz2 -d `ros2 pkg prefix minibot_navigation2`/share/minibot_navigation2/rviz/nav2_view.rviz
```


### For Real Robot Mapping
```shell
$ ros2 launch minibot_bringup bringup_robot.launch.xml
```
```shell
$ ros2 launch minibot_navigation2 map_building.launch.xml
```
```shell
$ rviz2 -d `ros2 pkg prefix minibot_navigation2`/share/minibot_navigation2/rviz/map_building.rviz
```
```shell
$ ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### For Navigation (Real Robot)
```shell
$ ros2 launch minibot_bringup bringup_robot.launch.xml
```
```shell
$ ros2 launch minibot_navigation2 bringup_launch.py map:=`ros2 pkg prefix minibot_navigation2`/share/minibot_navigation2/maps/<map-name>.yaml
```
```shell
$ rviz2 -d `ros2 pkg prefix minibot_navigation2`/share/minibot_navigation2/rviz/nav2_view.rviz
```
