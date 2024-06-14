;; Auto-generated. Do not edit!


(when (boundp 'husky_gazebo_plugins::GetHuskyOdometry)
  (if (not (find-package "HUSKY_GAZEBO_PLUGINS"))
    (make-package "HUSKY_GAZEBO_PLUGINS"))
  (shadow 'GetHuskyOdometry (find-package "HUSKY_GAZEBO_PLUGINS")))
(unless (find-package "HUSKY_GAZEBO_PLUGINS::GETHUSKYODOMETRY")
  (make-package "HUSKY_GAZEBO_PLUGINS::GETHUSKYODOMETRY"))
(unless (find-package "HUSKY_GAZEBO_PLUGINS::GETHUSKYODOMETRYREQUEST")
  (make-package "HUSKY_GAZEBO_PLUGINS::GETHUSKYODOMETRYREQUEST"))
(unless (find-package "HUSKY_GAZEBO_PLUGINS::GETHUSKYODOMETRYRESPONSE")
  (make-package "HUSKY_GAZEBO_PLUGINS::GETHUSKYODOMETRYRESPONSE"))

(in-package "ROS")



(if (not (find-package "NAV_MSGS"))
  (ros::roseus-add-msgs "nav_msgs"))


(defclass husky_gazebo_plugins::GetHuskyOdometryRequest
  :super ros::object
  :slots ())

(defmethod husky_gazebo_plugins::GetHuskyOdometryRequest
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

(defclass husky_gazebo_plugins::GetHuskyOdometryResponse
  :super ros::object
  :slots (_odometry _success _status_message ))

(defmethod husky_gazebo_plugins::GetHuskyOdometryResponse
  (:init
   (&key
    ((:odometry __odometry) (instance nav_msgs::Odometry :init))
    ((:success __success) nil)
    ((:status_message __status_message) "")
    )
   (send-super :init)
   (setq _odometry __odometry)
   (setq _success __success)
   (setq _status_message (string __status_message))
   self)
  (:odometry
   (&rest __odometry)
   (if (keywordp (car __odometry))
       (send* _odometry __odometry)
     (progn
       (if __odometry (setq _odometry (car __odometry)))
       _odometry)))
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:status_message
   (&optional __status_message)
   (if __status_message (setq _status_message __status_message)) _status_message)
  (:serialization-length
   ()
   (+
    ;; nav_msgs/Odometry _odometry
    (send _odometry :serialization-length)
    ;; bool _success
    1
    ;; string _status_message
    4 (length _status_message)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; nav_msgs/Odometry _odometry
       (send _odometry :serialize s)
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _status_message
       (write-long (length _status_message) s) (princ _status_message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; nav_msgs/Odometry _odometry
     (send _odometry :deserialize buf ptr-) (incf ptr- (send _odometry :serialization-length))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _status_message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _status_message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass husky_gazebo_plugins::GetHuskyOdometry
  :super ros::object
  :slots ())

(setf (get husky_gazebo_plugins::GetHuskyOdometry :md5sum-) "34929f1f9b719059cb41724dedcf33c1")
(setf (get husky_gazebo_plugins::GetHuskyOdometry :datatype-) "husky_gazebo_plugins/GetHuskyOdometry")
(setf (get husky_gazebo_plugins::GetHuskyOdometry :request) husky_gazebo_plugins::GetHuskyOdometryRequest)
(setf (get husky_gazebo_plugins::GetHuskyOdometry :response) husky_gazebo_plugins::GetHuskyOdometryResponse)

(defmethod husky_gazebo_plugins::GetHuskyOdometryRequest
  (:response () (instance husky_gazebo_plugins::GetHuskyOdometryResponse :init)))

(setf (get husky_gazebo_plugins::GetHuskyOdometryRequest :md5sum-) "34929f1f9b719059cb41724dedcf33c1")
(setf (get husky_gazebo_plugins::GetHuskyOdometryRequest :datatype-) "husky_gazebo_plugins/GetHuskyOdometryRequest")
(setf (get husky_gazebo_plugins::GetHuskyOdometryRequest :definition-)
      "---
nav_msgs/Odometry odometry
bool success                         # return true if get successful
string status_message                # comments if available


================================================================================
MSG: nav_msgs/Odometry
# This represents an estimate of a position and velocity in free space.  
# The pose in this message should be specified in the coordinate frame given by header.frame_id.
# The twist in this message should be specified in the coordinate frame given by the child_frame_id
Header header
string child_frame_id
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist

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

================================================================================
MSG: geometry_msgs/PoseWithCovariance
# This represents a pose in free space with uncertainty.

Pose pose

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: geometry_msgs/TwistWithCovariance
# This expresses velocity in free space with uncertainty.

Twist twist

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance

================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
")

(setf (get husky_gazebo_plugins::GetHuskyOdometryResponse :md5sum-) "34929f1f9b719059cb41724dedcf33c1")
(setf (get husky_gazebo_plugins::GetHuskyOdometryResponse :datatype-) "husky_gazebo_plugins/GetHuskyOdometryResponse")
(setf (get husky_gazebo_plugins::GetHuskyOdometryResponse :definition-)
      "---
nav_msgs/Odometry odometry
bool success                         # return true if get successful
string status_message                # comments if available


================================================================================
MSG: nav_msgs/Odometry
# This represents an estimate of a position and velocity in free space.  
# The pose in this message should be specified in the coordinate frame given by header.frame_id.
# The twist in this message should be specified in the coordinate frame given by the child_frame_id
Header header
string child_frame_id
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist

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

================================================================================
MSG: geometry_msgs/PoseWithCovariance
# This represents a pose in free space with uncertainty.

Pose pose

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: geometry_msgs/TwistWithCovariance
# This expresses velocity in free space with uncertainty.

Twist twist

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance

================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
")



(provide :husky_gazebo_plugins/GetHuskyOdometry "34929f1f9b719059cb41724dedcf33c1")


