roscore & sleep 1 && source ROS_Plugins/catkin_ws/devel/setup.bash && rosparam set use_sim_time true && rosrun gazebo_ros gzserver --verbose ./maze.sdf
