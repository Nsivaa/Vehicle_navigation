; Auto-generated. Do not edit!


(cl:in-package husky_gazebo_plugins-srv)


;//! \htmlinclude GetHuskyJointStates-request.msg.html

(cl:defclass <GetHuskyJointStates-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetHuskyJointStates-request (<GetHuskyJointStates-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetHuskyJointStates-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetHuskyJointStates-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name husky_gazebo_plugins-srv:<GetHuskyJointStates-request> is deprecated: use husky_gazebo_plugins-srv:GetHuskyJointStates-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetHuskyJointStates-request>) ostream)
  "Serializes a message object of type '<GetHuskyJointStates-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetHuskyJointStates-request>) istream)
  "Deserializes a message object of type '<GetHuskyJointStates-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetHuskyJointStates-request>)))
  "Returns string type for a service object of type '<GetHuskyJointStates-request>"
  "husky_gazebo_plugins/GetHuskyJointStatesRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetHuskyJointStates-request)))
  "Returns string type for a service object of type 'GetHuskyJointStates-request"
  "husky_gazebo_plugins/GetHuskyJointStatesRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetHuskyJointStates-request>)))
  "Returns md5sum for a message object of type '<GetHuskyJointStates-request>"
  "f69960c563cc6eff60e989900150b704")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetHuskyJointStates-request)))
  "Returns md5sum for a message object of type 'GetHuskyJointStates-request"
  "f69960c563cc6eff60e989900150b704")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetHuskyJointStates-request>)))
  "Returns full string definition for message of type '<GetHuskyJointStates-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetHuskyJointStates-request)))
  "Returns full string definition for message of type 'GetHuskyJointStates-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetHuskyJointStates-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetHuskyJointStates-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetHuskyJointStates-request
))
;//! \htmlinclude GetHuskyJointStates-response.msg.html

(cl:defclass <GetHuskyJointStates-response> (roslisp-msg-protocol:ros-message)
  ((joint_states
    :reader joint_states
    :initarg :joint_states
    :type sensor_msgs-msg:JointState
    :initform (cl:make-instance 'sensor_msgs-msg:JointState))
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

(cl:defclass GetHuskyJointStates-response (<GetHuskyJointStates-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetHuskyJointStates-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetHuskyJointStates-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name husky_gazebo_plugins-srv:<GetHuskyJointStates-response> is deprecated: use husky_gazebo_plugins-srv:GetHuskyJointStates-response instead.")))

(cl:ensure-generic-function 'joint_states-val :lambda-list '(m))
(cl:defmethod joint_states-val ((m <GetHuskyJointStates-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader husky_gazebo_plugins-srv:joint_states-val is deprecated.  Use husky_gazebo_plugins-srv:joint_states instead.")
  (joint_states m))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <GetHuskyJointStates-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader husky_gazebo_plugins-srv:success-val is deprecated.  Use husky_gazebo_plugins-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'status_message-val :lambda-list '(m))
(cl:defmethod status_message-val ((m <GetHuskyJointStates-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader husky_gazebo_plugins-srv:status_message-val is deprecated.  Use husky_gazebo_plugins-srv:status_message instead.")
  (status_message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetHuskyJointStates-response>) ostream)
  "Serializes a message object of type '<GetHuskyJointStates-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'joint_states) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'status_message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'status_message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetHuskyJointStates-response>) istream)
  "Deserializes a message object of type '<GetHuskyJointStates-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'joint_states) istream)
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetHuskyJointStates-response>)))
  "Returns string type for a service object of type '<GetHuskyJointStates-response>"
  "husky_gazebo_plugins/GetHuskyJointStatesResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetHuskyJointStates-response)))
  "Returns string type for a service object of type 'GetHuskyJointStates-response"
  "husky_gazebo_plugins/GetHuskyJointStatesResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetHuskyJointStates-response>)))
  "Returns md5sum for a message object of type '<GetHuskyJointStates-response>"
  "f69960c563cc6eff60e989900150b704")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetHuskyJointStates-response)))
  "Returns md5sum for a message object of type 'GetHuskyJointStates-response"
  "f69960c563cc6eff60e989900150b704")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetHuskyJointStates-response>)))
  "Returns full string definition for message of type '<GetHuskyJointStates-response>"
  (cl:format cl:nil "sensor_msgs/JointState joint_states~%bool success                         # return true if get successful~%string status_message                # comments if available~%~%~%================================================================================~%MSG: sensor_msgs/JointState~%# This is a message that holds data to describe the state of a set of torque controlled joints. ~%#~%# The state of each joint (revolute or prismatic) is defined by:~%#  * the position of the joint (rad or m),~%#  * the velocity of the joint (rad/s or m/s) and ~%#  * the effort that is applied in the joint (Nm or N).~%#~%# Each joint is uniquely identified by its name~%# The header specifies the time at which the joint states were recorded. All the joint states~%# in one message have to be recorded at the same time.~%#~%# This message consists of a multiple arrays, one for each part of the joint state. ~%# The goal is to make each of the fields optional. When e.g. your joints have no~%# effort associated with them, you can leave the effort array empty. ~%#~%# All arrays in this message should have the same size, or be empty.~%# This is the only way to uniquely associate the joint name with the correct~%# states.~%~%~%Header header~%~%string[] name~%float64[] position~%float64[] velocity~%float64[] effort~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetHuskyJointStates-response)))
  "Returns full string definition for message of type 'GetHuskyJointStates-response"
  (cl:format cl:nil "sensor_msgs/JointState joint_states~%bool success                         # return true if get successful~%string status_message                # comments if available~%~%~%================================================================================~%MSG: sensor_msgs/JointState~%# This is a message that holds data to describe the state of a set of torque controlled joints. ~%#~%# The state of each joint (revolute or prismatic) is defined by:~%#  * the position of the joint (rad or m),~%#  * the velocity of the joint (rad/s or m/s) and ~%#  * the effort that is applied in the joint (Nm or N).~%#~%# Each joint is uniquely identified by its name~%# The header specifies the time at which the joint states were recorded. All the joint states~%# in one message have to be recorded at the same time.~%#~%# This message consists of a multiple arrays, one for each part of the joint state. ~%# The goal is to make each of the fields optional. When e.g. your joints have no~%# effort associated with them, you can leave the effort array empty. ~%#~%# All arrays in this message should have the same size, or be empty.~%# This is the only way to uniquely associate the joint name with the correct~%# states.~%~%~%Header header~%~%string[] name~%float64[] position~%float64[] velocity~%float64[] effort~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetHuskyJointStates-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'joint_states))
     1
     4 (cl:length (cl:slot-value msg 'status_message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetHuskyJointStates-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetHuskyJointStates-response
    (cl:cons ':joint_states (joint_states msg))
    (cl:cons ':success (success msg))
    (cl:cons ':status_message (status_message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetHuskyJointStates)))
  'GetHuskyJointStates-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetHuskyJointStates)))
  'GetHuskyJointStates-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetHuskyJointStates)))
  "Returns string type for a service object of type '<GetHuskyJointStates>"
  "husky_gazebo_plugins/GetHuskyJointStates")