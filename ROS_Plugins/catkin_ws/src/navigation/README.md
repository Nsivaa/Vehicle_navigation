This is a ROS package for the navigation of a Braitenberg Vehicle in a space. The vehicle will detect red lights and will move towards them. 
When they change colour (or if no red shapes are found), the vehicle will spin until it finds one.

The package consists of two nodes: the first is **cam_subscriber**, which gathers camera information from the husky topic, processes it with OpenCv and publishes the result to the ```img_result``` topic. The result is formatted as a custom message defined in the ```/msg``` folder. 
The ```img_result``` message consists of a string field and an integer field: the first acts as a boolean to describe if red was detected in the corresponding image. The second field describes the horizontal disalignment of the center of the red shape with respect to the center of the image/camera. The "boolean" field is needed to avoid encoding "no red light detected" information just in the shift: a very arbitrary big value could have been chosen (e.g. -10000), but would have been a poor design choice because a higher resolution image could actually include that value as a valid shift. 

The second node, **vel_publisher**, reads the result from the ```img_result``` topic and publishes the appropriate velocity accordingly, on the ```cmd_vel``` topic. In case the image doesn't contain red, we spin left with a velocity of 3 (left because it's more convenient in the simulation map: there are more turns left than right to be done). In case it does, but the vehicle is not aligned to its center (therefore not "ready" to move forward), we rotate the veichle towards it, with a velocity inversely proportional to the shift (logarithmically). This to avoid overshooting. The vehicle is considered "aligned" with the image if the shift is below a certain threshold, because otherwise it would have to rotationally re-adjust at every step forward. 

All the parameters used by the node are fetched by the *.yaml* file in the ```config/``` directory. 

The default publishing rate is 20, to match the camera framerate, which was found using the ``` rostopic hz ``` command. 


## *RUNNING*


To run the package, use the following commands:

- Setup the catkin environment (in the ```catkin_ws``` folder)
  ``` {.bash}
    catkin_make
  ```
- Run the catkin setup script **for every terminal instance**
  ``` {.bash}
    source catkin_ws/devel/setup.bash
  ```

- Start roscore
  ``` {.bash}
  roscore & sleep 1
  rosparam set use_sim_time true # use gazebo simulated clock as ROS Time
  ```
- Run gazebo
  ``` {.bash}
  rosrun gazebo_ros gzserver --verbose <PATH TO world sdf file>
  ```

- Run the gazebo client **in a new terminal instance**
  ``` {.bash}
  gzclient --verbose 
  ```

- Run the SMACH State Machines that switch the lights from blue to red (and vice-versa) **in a new terminal instance**.
  ``` {.bash}
  python3 -m sm.lights_switcher # ctrl-c to terminate the scripts
  ```

- Run the launch file **in a new terminal instance**.  
    ``` {.bash}
    roslaunch navigation navigation.launch [args]
    ```
  
  ### Roslaunch Arguments
  ``` [args] ``` can be passed to override the following ROS parameters (they are set to False by default):
  - ``` verbose:=True ```: prints messages in execution logs
  - ``` debug:=True ```: save a copy of the camera images (one every 10) in the ``` /debug ``` folder, both raw and with the detected contour drawn on top. 

