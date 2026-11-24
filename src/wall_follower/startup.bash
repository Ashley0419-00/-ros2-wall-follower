ros2 launch turtlebot3_gazebo turtlebot3_maze.launch.py &
sleep 20
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True &
sleep 25
ros2 launch wall_follower wall_follower.launch.py use_sim_time:=true &
