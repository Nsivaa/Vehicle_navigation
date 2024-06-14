;; Auto-generated. Do not edit!


(when (boundp 'husky_gazebo_plugins::GetHuskyJointStates)
  (if (not (find-package "HUSKY_GAZEBO_PLUGINS"))
    (make-package "HUSKY_GAZEBO_PLUGINS"))
  (shadow 'GetHuskyJointStates (find-package "HUSKY_GAZEBO_PLUGINS")))
(unless (find-package "HUSKY_GAZEBO_PLUGINS::GETHUSKYJOINTSTATES")
  (make-package "HUSKY_GAZEBO_PLUGINS::GETHUSKYJOINTSTATES"))
(unless (find-package "HUSKY_GAZEBO_PLUGINS::GETHUSKYJOINTSTATESREQUEST")
  (make-package "HUSKY_GAZEBO_PLUGINS::GETHUSKYJOINTSTATESREQUEST"))
(unless (find-package "HUSKY_GAZEBO_PLUGINS::GETHUSKYJOINTSTATESRESPONSE")
  (make-package "HUSKY_GAZEBO_PLUGINS::GETHUSKYJOINTSTATESRESPONSE"))

(in-package "ROS")



(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))


(defclass husky_gazebo_plugins::GetHuskyJointStatesRequest
  :super ros::object
  :slots ())

(defmethod husky_gazebo_plugins::GetHuskyJointStatesRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass husky_gazebo_plugins::GetHuskyJointStatesResponse
  :super ros::object
  :slots (_joint_states _success _status_message ))

(defmethod husky_gazebo_plugins::GetHuskyJointStatesResponse
  (:init
   (&key
    ((:joint_states __joint_states) (instance sensor_msgs::JointState :init))
    ((:success __success) nil)
    ((:status_message __status_message) "")
    )
   (send-super :init)
   (setq _joint_states __joint_states)
   (setq _success __success)
   (setq _status_message (string __status_message))
   self)
  (:joint_states
   (&rest __joint_states)
   (if (keywordp (car __joint_states))
       (send* _joint_states __joint_states)
     (progn
       (if __joint_states (setq _joint_states (car __joint_states)))
       _joint_states)))
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:status_message
   (&optional __status_message)
   (if __status_message (setq _status_message __status_message)) _status_message)
  (:serialization-length
   ()
   (+
    ;; sensor_msgs/JointState _joint_states
    (send _joint_states :serialization-length)
    ;; bool _success
    1
    ;; string _status_message
    4 (length _status_message)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; sensor_msgs/JointState _joint_states
       (send _joint_states :serialize s)
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _status_message
       (write-long (length _status_message) s) (princ _status_message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; sensor_msgs/JointState _joint_states
     (send _joint_states :deserialize buf ptr-) (incf ptr- (send _joint_states :serialization-length))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _status_message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _status_message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass husky_gazebo_plugins::GetHuskyJointStates
  :super ros::object
  :slots ())

(setf (get husky_gazebo_plugins::GetHuskyJointStates :md5sum-) "f69960c563cc6eff60e989900150b704")
(setf (get husky_gazebo_plugins::GetHuskyJointStates :datatype-) "husky_gazebo_plugins/GetHuskyJointStates")
(setf (get husky_gazebo_plugins::GetHuskyJointStates :request) husky_gazebo_plugins::GetHuskyJointStatesRequest)
(setf (get husky_gazebo_plugins::GetHuskyJointStates :response) husky_gazebo_plugins::GetHuskyJointStatesResponse)

(defmethod husky_gazebo_plugins::GetHuskyJointStatesRequest
  (:response () (instance husky_gazebo_plugins::GetHuskyJointStatesResponse :init)))

(setf (get husky_gazebo_plugins::GetHuskyJointStatesRequest :md5sum-) "f69960c563cc6eff60e989900150b704")
(setf (get husky_gazebo_plugins::GetHuskyJointStatesRequest :datatype-) "husky_gazebo_plugins/GetHuskyJointStatesRequest")
(setf (get husky_gazebo_plugins::GetHuskyJointStatesRequest :definition-)
      "---
sensor_msgs/JointState joint_states
bool success                         # return true if get successful
string status_message                # comments if available


================================================================================
MSG: sensor_msgs/JointState
# This is a message that holds data to describe the state of a set of torque controlled joints. 
#
# The state of each joint (revolute or prismatic) is defined by:
#  * the position of the joint (rad or m),
#  * the velocity of the joint (rad/s or m/s) and 
#  * the effort that is applied in the joint (Nm or N).
#
# Each joint is uniquely identified by its name
# The header specifies the time at which the joint states were recorded. All the joint states
# in one message have to be recorded at the same time.
#
# This message consists of a multiple arrays, one for each part of the joint state. 
# The goal is to make each of the fields optional. When e.g. your joints have no
# effort associated with them, you can leave the effort array empty. 
#
# All arrays in this message should have the same size, or be empty.
# This is the only way to uniquely associate the joint name with the correct
# states.


Header header

string[] name
float64[] position
float64[] velocity
float64[] effort

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id
")

(setf (get husky_gazebo_plugins::GetHuskyJointStatesResponse :md5sum-) "f69960c563cc6eff60e989900150b704")
(setf (get husky_gazebo_plugins::GetHuskyJointStatesResponse :datatype-) "husky_gazebo_plugins/GetHuskyJointStatesResponse")
(setf (get husky_gazebo_plugins::GetHuskyJointStatesResponse :definition-)
      "---
sensor_msgs/JointState joint_states
bool success                         # return true if get successful
string status_message                # comments if available


================================================================================
MSG: sensor_msgs/JointState
# This is a message that holds data to describe the state of a set of torque controlled joints. 
#
# The state of each joint (revolute or prismatic) is defined by:
#  * the position of the joint (rad or m),
#  * the velocity of the joint (rad/s or m/s) and 
#  * the effort that is applied in the joint (Nm or N).
#
# Each joint is uniquely identified by its name
# The header specifies the time at which the joint states were recorded. All the joint states
# in one message have to be recorded at the same time.
#
# This message consists of a multiple arrays, one for each part of the joint state. 
# The goal is to make each of the fields optional. When e.g. your joints have no
# effort associated with them, you can leave the effort array empty. 
#
# All arrays in this message should have the same size, or be empty.
# This is the only way to uniquely associate the joint name with the correct
# states.


Header header

string[] name
float64[] position
float64[] velocity
float64[] effort

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id
")



(provide :husky_gazebo_plugins/GetHuskyJointStates "f69960c563cc6eff60e989900150b704")


