# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "husky_gazebo_plugins: 1 messages, 4 services")

set(MSG_I_FLAGS "-Ihusky_gazebo_plugins:/home/nsiva/braitenberg_nav_project/ROS_Plugins/catkin_ws/src/husky_gazebo_plugins/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Inav_msgs:/opt/ros/noetic/share/nav_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/noetic/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(husky_gazebo_plugins_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/nsiva/braitenberg_nav_project/ROS_Plugins/catkin_ws/src/husky_gazebo_plugins/msg/WheelSpeeds.msg" NAME_WE)
add_custom_target(_husky_gazebo_plugins_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "husky_gazebo_plugins" "/home/nsiva/braitenberg_nav_project/ROS_Plugins/catkin_ws/src/husky_gazebo_plugins/msg/WheelSpeeds.msg" ""
)

get_filename_component(_filename "/home/nsiva/braitenberg_nav_project/ROS_Plugins/catkin_ws/src/husky_gazebo_plugins/srv/SetHuskyCmdVel.srv" NAME_WE)
add_custom_target(_husky_gazebo_plugins_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "husky_gazebo_plugins" "/home/nsiva/braitenberg_nav_project/ROS_Plugins/catkin_ws/src/husky_gazebo_plugins/srv/SetHuskyCmdVel.srv" "geometry_msgs/Twist:geometry_msgs/Vector3"
)

get_filename_component(_filename "/home/nsiva/braitenberg_nav_project/ROS_Plugins/catkin_ws/src/husky_gazebo_plugins/srv/GetHuskyOdometry.srv" NAME_WE)
add_custom_target(_husky_gazebo_plugins_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "husky_gazebo_plugins" "/home/nsiva/braitenberg_nav_project/ROS_Plugins/catkin_ws/src/husky_gazebo_plugins/srv/GetHuskyOdometry.srv" "geometry_msgs/Quaternion:nav_msgs/Odometry:std_msgs/Header:geometry_msgs/Twist:geometry_msgs/Point:geometry_msgs/TwistWithCovariance:geometry_msgs/PoseWithCovariance:geometry_msgs/Pose:geometry_msgs/Vector3"
)

get_filename_component(_filename "/home/nsiva/braitenberg_nav_project/ROS_Plugins/catkin_ws/src/husky_gazebo_plugins/srv/GetHuskyJointStates.srv" NAME_WE)
add_custom_target(_husky_gazebo_plugins_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "husky_gazebo_plugins" "/home/nsiva/braitenberg_nav_project/ROS_Plugins/catkin_ws/src/husky_gazebo_plugins/srv/GetHuskyJointStates.srv" "std_msgs/Header:sensor_msgs/JointState"
)

get_filename_component(_filename "/home/nsiva/braitenberg_nav_project/ROS_Plugins/catkin_ws/src/husky_gazebo_plugins/srv/SetHuskyWheelSpeeds.srv" NAME_WE)
add_custom_target(_husky_gazebo_plugins_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "husky_gazebo_plugins" "/home/nsiva/braitenberg_nav_project/ROS_Plugins/catkin_ws/src/husky_gazebo_plugins/srv/SetHuskyWheelSpeeds.srv" "husky_gazebo_plugins/WheelSpeeds"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(husky_gazebo_plugins
  "/home/nsiva/braitenberg_nav_project/ROS_Plugins/catkin_ws/src/husky_gazebo_plugins/msg/WheelSpeeds.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/husky_gazebo_plugins
)

### Generating Services
_generate_srv_cpp(husky_gazebo_plugins
  "/home/nsiva/braitenberg_nav_project/ROS_Plugins/catkin_ws/src/husky_gazebo_plugins/srv/SetHuskyCmdVel.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/husky_gazebo_plugins
)
_generate_srv_cpp(husky_gazebo_plugins
  "/home/nsiva/braitenberg_nav_project/ROS_Plugins/catkin_ws/src/husky_gazebo_plugins/srv/GetHuskyOdometry.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/nav_msgs/cmake/../msg/Odometry.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/husky_gazebo_plugins
)
_generate_srv_cpp(husky_gazebo_plugins
  "/home/nsiva/braitenberg_nav_project/ROS_Plugins/catkin_ws/src/husky_gazebo_plugins/srv/GetHuskyJointStates.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/JointState.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/husky_gazebo_plugins
)
_generate_srv_cpp(husky_gazebo_plugins
  "/home/nsiva/braitenberg_nav_project/ROS_Plugins/catkin_ws/src/husky_gazebo_plugins/srv/SetHuskyWheelSpeeds.srv"
  "${MSG_I_FLAGS}"
  "/home/nsiva/braitenberg_nav_project/ROS_Plugins/catkin_ws/src/husky_gazebo_plugins/msg/WheelSpeeds.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/husky_gazebo_plugins
)

### Generating Module File
_generate_module_cpp(husky_gazebo_plugins
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/husky_gazebo_plugins
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(husky_gazebo_plugins_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(husky_gazebo_plugins_generate_messages husky_gazebo_plugins_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nsiva/braitenberg_nav_project/ROS_Plugins/catkin_ws/src/husky_gazebo_plugins/msg/WheelSpeeds.msg" NAME_WE)
add_dependencies(husky_gazebo_plugins_generate_messages_cpp _husky_gazebo_plugins_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nsiva/braitenberg_nav_project/ROS_Plugins/catkin_ws/src/husky_gazebo_plugins/srv/SetHuskyCmdVel.srv" NAME_WE)
add_dependencies(husky_gazebo_plugins_generate_messages_cpp _husky_gazebo_plugins_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nsiva/braitenberg_nav_project/ROS_Plugins/catkin_ws/src/husky_gazebo_plugins/srv/GetHuskyOdometry.srv" NAME_WE)
add_dependencies(husky_gazebo_plugins_generate_messages_cpp _husky_gazebo_plugins_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nsiva/braitenberg_nav_project/ROS_Plugins/catkin_ws/src/husky_gazebo_plugins/srv/GetHuskyJointStates.srv" NAME_WE)
add_dependencies(husky_gazebo_plugins_generate_messages_cpp _husky_gazebo_plugins_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nsiva/braitenberg_nav_project/ROS_Plugins/catkin_ws/src/husky_gazebo_plugins/srv/SetHuskyWheelSpeeds.srv" NAME_WE)
add_dependencies(husky_gazebo_plugins_generate_messages_cpp _husky_gazebo_plugins_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(husky_gazebo_plugins_gencpp)
add_dependencies(husky_gazebo_plugins_gencpp husky_gazebo_plugins_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS husky_gazebo_plugins_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(husky_gazebo_plugins
  "/home/nsiva/braitenberg_nav_project/ROS_Plugins/catkin_ws/src/husky_gazebo_plugins/msg/WheelSpeeds.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/husky_gazebo_plugins
)

### Generating Services
_generate_srv_eus(husky_gazebo_plugins
  "/home/nsiva/braitenberg_nav_project/ROS_Plugins/catkin_ws/src/husky_gazebo_plugins/srv/SetHuskyCmdVel.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/husky_gazebo_plugins
)
_generate_srv_eus(husky_gazebo_plugins
  "/home/nsiva/braitenberg_nav_project/ROS_Plugins/catkin_ws/src/husky_gazebo_plugins/srv/GetHuskyOdometry.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/nav_msgs/cmake/../msg/Odometry.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/husky_gazebo_plugins
)
_generate_srv_eus(husky_gazebo_plugins
  "/home/nsiva/braitenberg_nav_project/ROS_Plugins/catkin_ws/src/husky_gazebo_plugins/srv/GetHuskyJointStates.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/JointState.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/husky_gazebo_plugins
)
_generate_srv_eus(husky_gazebo_plugins
  "/home/nsiva/braitenberg_nav_project/ROS_Plugins/catkin_ws/src/husky_gazebo_plugins/srv/SetHuskyWheelSpeeds.srv"
  "${MSG_I_FLAGS}"
  "/home/nsiva/braitenberg_nav_project/ROS_Plugins/catkin_ws/src/husky_gazebo_plugins/msg/WheelSpeeds.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/husky_gazebo_plugins
)

### Generating Module File
_generate_module_eus(husky_gazebo_plugins
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/husky_gazebo_plugins
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(husky_gazebo_plugins_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(husky_gazebo_plugins_generate_messages husky_gazebo_plugins_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nsiva/braitenberg_nav_project/ROS_Plugins/catkin_ws/src/husky_gazebo_plugins/msg/WheelSpeeds.msg" NAME_WE)
add_dependencies(husky_gazebo_plugins_generate_messages_eus _husky_gazebo_plugins_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nsiva/braitenberg_nav_project/ROS_Plugins/catkin_ws/src/husky_gazebo_plugins/srv/SetHuskyCmdVel.srv" NAME_WE)
add_dependencies(husky_gazebo_plugins_generate_messages_eus _husky_gazebo_plugins_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nsiva/braitenberg_nav_project/ROS_Plugins/catkin_ws/src/husky_gazebo_plugins/srv/GetHuskyOdometry.srv" NAME_WE)
add_dependencies(husky_gazebo_plugins_generate_messages_eus _husky_gazebo_plugins_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nsiva/braitenberg_nav_project/ROS_Plugins/catkin_ws/src/husky_gazebo_plugins/srv/GetHuskyJointStates.srv" NAME_WE)
add_dependencies(husky_gazebo_plugins_generate_messages_eus _husky_gazebo_plugins_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nsiva/braitenberg_nav_project/ROS_Plugins/catkin_ws/src/husky_gazebo_plugins/srv/SetHuskyWheelSpeeds.srv" NAME_WE)
add_dependencies(husky_gazebo_plugins_generate_messages_eus _husky_gazebo_plugins_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(husky_gazebo_plugins_geneus)
add_dependencies(husky_gazebo_plugins_geneus husky_gazebo_plugins_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS husky_gazebo_plugins_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(husky_gazebo_plugins
  "/home/nsiva/braitenberg_nav_project/ROS_Plugins/catkin_ws/src/husky_gazebo_plugins/msg/WheelSpeeds.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/husky_gazebo_plugins
)

### Generating Services
_generate_srv_lisp(husky_gazebo_plugins
  "/home/nsiva/braitenberg_nav_project/ROS_Plugins/catkin_ws/src/husky_gazebo_plugins/srv/SetHuskyCmdVel.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/husky_gazebo_plugins
)
_generate_srv_lisp(husky_gazebo_plugins
  "/home/nsiva/braitenberg_nav_project/ROS_Plugins/catkin_ws/src/husky_gazebo_plugins/srv/GetHuskyOdometry.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/nav_msgs/cmake/../msg/Odometry.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/husky_gazebo_plugins
)
_generate_srv_lisp(husky_gazebo_plugins
  "/home/nsiva/braitenberg_nav_project/ROS_Plugins/catkin_ws/src/husky_gazebo_plugins/srv/GetHuskyJointStates.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/JointState.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/husky_gazebo_plugins
)
_generate_srv_lisp(husky_gazebo_plugins
  "/home/nsiva/braitenberg_nav_project/ROS_Plugins/catkin_ws/src/husky_gazebo_plugins/srv/SetHuskyWheelSpeeds.srv"
  "${MSG_I_FLAGS}"
  "/home/nsiva/braitenberg_nav_project/ROS_Plugins/catkin_ws/src/husky_gazebo_plugins/msg/WheelSpeeds.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/husky_gazebo_plugins
)

### Generating Module File
_generate_module_lisp(husky_gazebo_plugins
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/husky_gazebo_plugins
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(husky_gazebo_plugins_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(husky_gazebo_plugins_generate_messages husky_gazebo_plugins_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nsiva/braitenberg_nav_project/ROS_Plugins/catkin_ws/src/husky_gazebo_plugins/msg/WheelSpeeds.msg" NAME_WE)
add_dependencies(husky_gazebo_plugins_generate_messages_lisp _husky_gazebo_plugins_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nsiva/braitenberg_nav_project/ROS_Plugins/catkin_ws/src/husky_gazebo_plugins/srv/SetHuskyCmdVel.srv" NAME_WE)
add_dependencies(husky_gazebo_plugins_generate_messages_lisp _husky_gazebo_plugins_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nsiva/braitenberg_nav_project/ROS_Plugins/catkin_ws/src/husky_gazebo_plugins/srv/GetHuskyOdometry.srv" NAME_WE)
add_dependencies(husky_gazebo_plugins_generate_messages_lisp _husky_gazebo_plugins_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nsiva/braitenberg_nav_project/ROS_Plugins/catkin_ws/src/husky_gazebo_plugins/srv/GetHuskyJointStates.srv" NAME_WE)
add_dependencies(husky_gazebo_plugins_generate_messages_lisp _husky_gazebo_plugins_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nsiva/braitenberg_nav_project/ROS_Plugins/catkin_ws/src/husky_gazebo_plugins/srv/SetHuskyWheelSpeeds.srv" NAME_WE)
add_dependencies(husky_gazebo_plugins_generate_messages_lisp _husky_gazebo_plugins_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(husky_gazebo_plugins_genlisp)
add_dependencies(husky_gazebo_plugins_genlisp husky_gazebo_plugins_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS husky_gazebo_plugins_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(husky_gazebo_plugins
  "/home/nsiva/braitenberg_nav_project/ROS_Plugins/catkin_ws/src/husky_gazebo_plugins/msg/WheelSpeeds.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/husky_gazebo_plugins
)

### Generating Services
_generate_srv_nodejs(husky_gazebo_plugins
  "/home/nsiva/braitenberg_nav_project/ROS_Plugins/catkin_ws/src/husky_gazebo_plugins/srv/SetHuskyCmdVel.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/husky_gazebo_plugins
)
_generate_srv_nodejs(husky_gazebo_plugins
  "/home/nsiva/braitenberg_nav_project/ROS_Plugins/catkin_ws/src/husky_gazebo_plugins/srv/GetHuskyOdometry.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/nav_msgs/cmake/../msg/Odometry.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/husky_gazebo_plugins
)
_generate_srv_nodejs(husky_gazebo_plugins
  "/home/nsiva/braitenberg_nav_project/ROS_Plugins/catkin_ws/src/husky_gazebo_plugins/srv/GetHuskyJointStates.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/JointState.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/husky_gazebo_plugins
)
_generate_srv_nodejs(husky_gazebo_plugins
  "/home/nsiva/braitenberg_nav_project/ROS_Plugins/catkin_ws/src/husky_gazebo_plugins/srv/SetHuskyWheelSpeeds.srv"
  "${MSG_I_FLAGS}"
  "/home/nsiva/braitenberg_nav_project/ROS_Plugins/catkin_ws/src/husky_gazebo_plugins/msg/WheelSpeeds.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/husky_gazebo_plugins
)

### Generating Module File
_generate_module_nodejs(husky_gazebo_plugins
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/husky_gazebo_plugins
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(husky_gazebo_plugins_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(husky_gazebo_plugins_generate_messages husky_gazebo_plugins_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nsiva/braitenberg_nav_project/ROS_Plugins/catkin_ws/src/husky_gazebo_plugins/msg/WheelSpeeds.msg" NAME_WE)
add_dependencies(husky_gazebo_plugins_generate_messages_nodejs _husky_gazebo_plugins_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nsiva/braitenberg_nav_project/ROS_Plugins/catkin_ws/src/husky_gazebo_plugins/srv/SetHuskyCmdVel.srv" NAME_WE)
add_dependencies(husky_gazebo_plugins_generate_messages_nodejs _husky_gazebo_plugins_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nsiva/braitenberg_nav_project/ROS_Plugins/catkin_ws/src/husky_gazebo_plugins/srv/GetHuskyOdometry.srv" NAME_WE)
add_dependencies(husky_gazebo_plugins_generate_messages_nodejs _husky_gazebo_plugins_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nsiva/braitenberg_nav_project/ROS_Plugins/catkin_ws/src/husky_gazebo_plugins/srv/GetHuskyJointStates.srv" NAME_WE)
add_dependencies(husky_gazebo_plugins_generate_messages_nodejs _husky_gazebo_plugins_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nsiva/braitenberg_nav_project/ROS_Plugins/catkin_ws/src/husky_gazebo_plugins/srv/SetHuskyWheelSpeeds.srv" NAME_WE)
add_dependencies(husky_gazebo_plugins_generate_messages_nodejs _husky_gazebo_plugins_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(husky_gazebo_plugins_gennodejs)
add_dependencies(husky_gazebo_plugins_gennodejs husky_gazebo_plugins_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS husky_gazebo_plugins_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(husky_gazebo_plugins
  "/home/nsiva/braitenberg_nav_project/ROS_Plugins/catkin_ws/src/husky_gazebo_plugins/msg/WheelSpeeds.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/husky_gazebo_plugins
)

### Generating Services
_generate_srv_py(husky_gazebo_plugins
  "/home/nsiva/braitenberg_nav_project/ROS_Plugins/catkin_ws/src/husky_gazebo_plugins/srv/SetHuskyCmdVel.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/husky_gazebo_plugins
)
_generate_srv_py(husky_gazebo_plugins
  "/home/nsiva/braitenberg_nav_project/ROS_Plugins/catkin_ws/src/husky_gazebo_plugins/srv/GetHuskyOdometry.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/nav_msgs/cmake/../msg/Odometry.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/husky_gazebo_plugins
)
_generate_srv_py(husky_gazebo_plugins
  "/home/nsiva/braitenberg_nav_project/ROS_Plugins/catkin_ws/src/husky_gazebo_plugins/srv/GetHuskyJointStates.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/JointState.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/husky_gazebo_plugins
)
_generate_srv_py(husky_gazebo_plugins
  "/home/nsiva/braitenberg_nav_project/ROS_Plugins/catkin_ws/src/husky_gazebo_plugins/srv/SetHuskyWheelSpeeds.srv"
  "${MSG_I_FLAGS}"
  "/home/nsiva/braitenberg_nav_project/ROS_Plugins/catkin_ws/src/husky_gazebo_plugins/msg/WheelSpeeds.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/husky_gazebo_plugins
)

### Generating Module File
_generate_module_py(husky_gazebo_plugins
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/husky_gazebo_plugins
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(husky_gazebo_plugins_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(husky_gazebo_plugins_generate_messages husky_gazebo_plugins_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nsiva/braitenberg_nav_project/ROS_Plugins/catkin_ws/src/husky_gazebo_plugins/msg/WheelSpeeds.msg" NAME_WE)
add_dependencies(husky_gazebo_plugins_generate_messages_py _husky_gazebo_plugins_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nsiva/braitenberg_nav_project/ROS_Plugins/catkin_ws/src/husky_gazebo_plugins/srv/SetHuskyCmdVel.srv" NAME_WE)
add_dependencies(husky_gazebo_plugins_generate_messages_py _husky_gazebo_plugins_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nsiva/braitenberg_nav_project/ROS_Plugins/catkin_ws/src/husky_gazebo_plugins/srv/GetHuskyOdometry.srv" NAME_WE)
add_dependencies(husky_gazebo_plugins_generate_messages_py _husky_gazebo_plugins_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nsiva/braitenberg_nav_project/ROS_Plugins/catkin_ws/src/husky_gazebo_plugins/srv/GetHuskyJointStates.srv" NAME_WE)
add_dependencies(husky_gazebo_plugins_generate_messages_py _husky_gazebo_plugins_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nsiva/braitenberg_nav_project/ROS_Plugins/catkin_ws/src/husky_gazebo_plugins/srv/SetHuskyWheelSpeeds.srv" NAME_WE)
add_dependencies(husky_gazebo_plugins_generate_messages_py _husky_gazebo_plugins_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(husky_gazebo_plugins_genpy)
add_dependencies(husky_gazebo_plugins_genpy husky_gazebo_plugins_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS husky_gazebo_plugins_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/husky_gazebo_plugins)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/husky_gazebo_plugins
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(husky_gazebo_plugins_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(husky_gazebo_plugins_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET nav_msgs_generate_messages_cpp)
  add_dependencies(husky_gazebo_plugins_generate_messages_cpp nav_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(husky_gazebo_plugins_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/husky_gazebo_plugins)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/husky_gazebo_plugins
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(husky_gazebo_plugins_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(husky_gazebo_plugins_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET nav_msgs_generate_messages_eus)
  add_dependencies(husky_gazebo_plugins_generate_messages_eus nav_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(husky_gazebo_plugins_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/husky_gazebo_plugins)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/husky_gazebo_plugins
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(husky_gazebo_plugins_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(husky_gazebo_plugins_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET nav_msgs_generate_messages_lisp)
  add_dependencies(husky_gazebo_plugins_generate_messages_lisp nav_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(husky_gazebo_plugins_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/husky_gazebo_plugins)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/husky_gazebo_plugins
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(husky_gazebo_plugins_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(husky_gazebo_plugins_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET nav_msgs_generate_messages_nodejs)
  add_dependencies(husky_gazebo_plugins_generate_messages_nodejs nav_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(husky_gazebo_plugins_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/husky_gazebo_plugins)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/husky_gazebo_plugins\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/husky_gazebo_plugins
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(husky_gazebo_plugins_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(husky_gazebo_plugins_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET nav_msgs_generate_messages_py)
  add_dependencies(husky_gazebo_plugins_generate_messages_py nav_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(husky_gazebo_plugins_generate_messages_py sensor_msgs_generate_messages_py)
endif()
