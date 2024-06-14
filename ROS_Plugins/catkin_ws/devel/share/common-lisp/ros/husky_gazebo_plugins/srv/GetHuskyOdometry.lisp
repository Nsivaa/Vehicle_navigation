; Auto-generated. Do not edit!


(cl:in-package husky_gazebo_plugins-srv)


;//! \htmlinclude GetHuskyOdometry-request.msg.html

(cl:defclass <GetHuskyOdometry-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetHuskyOdometry-request (<GetHuskyOdometry-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetHuskyOdometry-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetHuskyOdometry-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name husky_gazebo_plugins-srv:<GetHuskyOdometry-request> is deprecated: use husky_gazebo_plugins-srv:GetHuskyOdometry-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetHuskyOdometry-request>) ostream)
  "Serializes a message object of type '<GetHuskyOdometry-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetHuskyOdometry-request>) istream)
  "Deserializes a message object of type '<GetHuskyOdometry-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetHuskyOdometry-request>)))
  "Returns string type for a service object of type '<GetHuskyOdometry-request>"
  "husky_gazebo_plugins/GetHuskyOdometryRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetHuskyOdometry-request)))
  "Returns string type for a service object of type 'GetHuskyOdometry-request"
  "husky_gazebo_plugins/GetHuskyOdometryRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetHuskyOdometry-request>)))
  "Returns md5sum for a message object of type '<GetHuskyOdometry-request>"
  "34929f1f9b719059cb41724dedcf33c1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetHuskyOdometry-request)))
  "Returns md5sum for a message object of type 'GetHuskyOdometry-request"
  "34929f1f9b719059cb41724dedcf33c1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetHuskyOdometry-request>)))
  "Returns full string definition for message of type '<GetHuskyOdometry-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetHuskyOdometry-request)))
  "Returns full string definition for message of type 'GetHuskyOdometry-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetHuskyOdometry-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetHuskyOdometry-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetHuskyOdometry-request
))
;//! \htmlinclude GetHuskyOdometry-response.msg.html

(cl:defclass <GetHuskyOdometry-response> (roslisp-msg-protocol:ros-message)
  ((odometry
    :reader odometry
    :initarg :odometry
    :type nav_msgs-msg:Odometry
    :initform (cl:make-instance 'nav_msgs-msg:Odometry))
   (success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (status_message
    :reader status_message
    :initarg :status_message
    :type cl:string
    :initform ""))
)

(cl:defclass GetHuskyOdometry-response (<GetHuskyOdometry-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetHuskyOdometry-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetHuskyOdometry-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name husky_gazebo_plugins-srv:<GetHuskyOdometry-response> is deprecated: use husky_gazebo_plugins-srv:GetHuskyOdometry-response instead.")))

(cl:ensure-generic-function 'odometry-val :lambda-list '(m))
(cl:defmethod odometry-val ((m <GetHuskyOdometry-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader husky_gazebo_plugins-srv:odometry-val is deprecated.  Use husky_gazebo_plugins-srv:odometry instead.")
  (odometry m))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <GetHuskyOdometry-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader husky_gazebo_plugins-srv:success-val is deprecated.  Use husky_gazebo_plugins-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'status_message-val :lambda-list '(m))
(cl:defmethod status_message-val ((m <GetHuskyOdometry-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader husky_gazebo_plugins-srv:status_message-val is deprecated.  Use husky_gazebo_plugins-srv:status_message instead.")
  (status_message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetHuskyOdometry-response>) ostream)
  "Serializes a message object of type '<GetHuskyOdometry-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'odometry) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'status_message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'status_message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetHuskyOdometry-response>) istream)
  "Deserializes a message object of type '<GetHuskyOdometry-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'odometry) istream)
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'status_message) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'status_message) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetHuskyOdometry-response>)))
  "Returns string type for a service object of type '<GetHuskyOdometry-response>"
  "husky_gazebo_plugins/GetHuskyOdometryResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetHuskyOdometry-response)))
  "Returns string type for a service object of type 'GetHuskyOdometry-response"
  "husky_gazebo_plugins/GetHuskyOdometryResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetHuskyOdometry-response>)))
  "Returns md5sum for a message object of type '<GetHuskyOdometry-response>"
  "34929f1f9b719059cb41724dedcf33c1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetHuskyOdometry-response)))
  "Returns md5sum for a message object of type 'GetHuskyOdometry-response"
  "34929f1f9b719059cb41724dedcf33c1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetHuskyOdometry-response>)))
  "Returns full string definition for message of type '<GetHuskyOdometry-response>"
  (cl:format cl:nil "nav_msgs/Odometry odometry~%bool success                         # return true if get successful~%string status_message                # comments if available~%~%~%================================================================================~%MSG: nav_msgs/Odometry~%# This represents an estimate of a position and velocity in free space.  ~%# The pose in this message should be specified in the coordinate frame given by header.frame_id.~%# The twist in this message should be specified in the coordinate frame given by the child_frame_id~%Header header~%string child_frame_id~%geometry_msgs/PoseWithCovariance pose~%geometry_msgs/TwistWithCovariance twist~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/PoseWithCovariance~%# This represents a pose in free space with uncertainty.~%~%Pose pose~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/TwistWithCovariance~%# This expresses velocity in free space with uncertainty.~%~%Twist twist~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetHuskyOdometry-response)))
  "Returns full string definition for message of type 'GetHuskyOdometry-response"
  (cl:format cl:nil "nav_msgs/Odometry odometry~%bool success                         # return true if get successful~%string status_message                # comments if available~%~%~%================================================================================~%MSG: nav_msgs/Odometry~%# This represents an estimate of a position and velocity in free space.  ~%# The pose in this message should be specified in the coordinate frame given by header.frame_id.~%# The twist in this message should be specified in the coordinate frame given by the child_frame_id~%Header header~%string child_frame_id~%geometry_msgs/PoseWithCovariance pose~%geometry_msgs/TwistWithCovariance twist~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/PoseWithCovariance~%# This represents a pose in free space with uncertainty.~%~%Pose pose~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/TwistWithCovariance~%# This expresses velocity in free space with uncertainty.~%~%Twist twist~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetHuskyOdometry-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'odometry))
     1
     4 (cl:length (cl:slot-value msg 'status_message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetHuskyOdometry-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetHuskyOdometry-response
    (cl:cons ':odometry (odometry msg))
    (cl:cons ':success (success msg))
    (cl:cons ':status_message (status_message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetHuskyOdometry)))
  'GetHuskyOdometry-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetHuskyOdometry)))
  'GetHuskyOdometry-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetHuskyOdometry)))
  "Returns string type for a service object of type '<GetHuskyOdometry>"
  "husky_gazebo_plugins/GetHuskyOdometry")