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

- Start roscore
  
    ``` {.bash}
    roscore & sleep 1
    rosparam set use_sim_time true # use gazebo simulated clock as ROS Time
    ```

- Run gazebo
    ``` {.bash}
    rosrun gazebo_ros gzserver --verbose <PATH TO world sdf file>
    gzclient --verbose # in a different terminal
    ```

- Run the SMACH State Machines that switch the lights from blue to red (and vice-versa).
    ``` {.bash}
    python3 -m sm.light_switcher # ctrl-c to terminate the scripts
    ```

The robot can be velocity controlled by publishing on */husky_model/husky/cmd_vel* topic (rostopic list to show other available topics)

Camera images are published on */husky_model/husky/camera*


**GOAL**

Design and implement robot navigation using reactive control based on robot camera images.

Implement one (or more) ROS Node(s) to control the robot, its goal is to navigate the maze following the path traced by the sequence of switching lamp posts. 

As soon as the robot approaches a working lamp post, its light switches to blue and the next one switches to red. 

