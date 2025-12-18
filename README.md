ros2 launch demobot2_description description.launch.py robot_type:=cubicbot
ros2 launch demobot2_description display.launch.py robot_type:=cubicbot

ros2 run nav2_map_server map_saver_cli -f ~/map