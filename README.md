**INSTALLATION**

- Copy the content of the Models directory to $HOME/.gazebo/models (create it if necessary)
  (On simulation loading gazebo looks for models in the the directories listed in the $GAZEBO_MODEL_PATH env var)

- Install Gazebo ROS Packages (ros-noetic-desktop is assumed to be installed) 
sudo apt-get install ros-noetic-gazebo-ros-pkgs 

- Build the workspace in ROS_Plugin\catkin_ws (catkin_make)
  ``` {.bash}
  source catkin_ws/devel/setup.bash # (else gazebo won't be able to find the plugins) 
  ```
**RUNNING**

1. Start roscore
  ``` {.bash}
  roscore & sleep 1
  rosparam set use_sim_time true # use gazebo simulated clock as ROS Time
  ```
2. Run gazebo
  ``` {.bash}
  rosrun gazebo_ros gzserver --verbose <PATH TO world sdf file>
  ```

- Run the gazebo client in a new terminal instance
  ``` {.bash}
  gzclient --verbose 
  ```

- Run the SMACH State Machines that switch the lights from blue to red (and vice-versa) in a new terminal instance.
  ``` {.bash}
  python3 -m sm.lights_switcher # ctrl-c to terminate the scripts
  ```

**GOAL**

Design and implement robot navigation using reactive control based on robot camera images.

Implement one (or more) ROS Node(s) to control the robot, its goal is to navigate the maze following the path traced by the sequence of switching lamp posts. 

As soon as the robot approaches a working lamp post, its light switches to blue and the next one switches to red. 

The repository contains a ROS package that implements a solution (ROS_Plugins/catkin_ws/src/navigation). Refer to its README file for info.
