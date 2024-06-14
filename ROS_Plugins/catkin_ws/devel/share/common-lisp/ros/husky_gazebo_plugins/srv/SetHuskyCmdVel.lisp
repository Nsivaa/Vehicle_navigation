; Auto-generated. Do not edit!


(cl:in-package husky_gazebo_plugins-srv)


;//! \htmlinclude SetHuskyCmdVel-request.msg.html

(cl:defclass <SetHuskyCmdVel-request> (roslisp-msg-protocol:ros-message)
  ((cmd_vel
    :reader cmd_vel
    :initarg :cmd_vel
    :type geometry_msgs-msg:Twist
    :initform (cl:make-instance 'geometry_msgs-msg:Twist)))
)

(cl:defclass SetHuskyCmdVel-request (<SetHuskyCmdVel-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetHuskyCmdVel-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetHuskyCmdVel-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name husky_gazebo_plugins-srv:<SetHuskyCmdVel-request> is deprecated: use husky_gazebo_plugins-srv:SetHuskyCmdVel-request instead.")))

(cl:ensure-generic-function 'cmd_vel-val :lambda-list '(m))
(cl:defmethod cmd_vel-val ((m <SetHuskyCmdVel-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader husky_gazebo_plugins-srv:cmd_vel-val is deprecated.  Use husky_gazebo_plugins-srv:cmd_vel instead.")
  (cmd_vel m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetHuskyCmdVel-request>) ostream)
  "Serializes a message object of type '<SetHuskyCmdVel-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'cmd_vel) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetHuskyCmdVel-request>) istream)
  "Deserializes a message object of type '<SetHuskyCmdVel-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'cmd_vel) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetHuskyCmdVel-request>)))
  "Returns string type for a service object of type '<SetHuskyCmdVel-request>"
  "husky_gazebo_plugins/SetHuskyCmdVelRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetHuskyCmdVel-request)))
  "Returns string type for a service object of type 'SetHuskyCmdVel-request"
  "husky_gazebo_plugins/SetHuskyCmdVelRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetHuskyCmdVel-request>)))
  "Returns md5sum for a message object of type '<SetHuskyCmdVel-request>"
  "cf6e6c41c08fa53df96254507e254422")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetHuskyCmdVel-request)))
  "Returns md5sum for a message object of type 'SetHuskyCmdVel-request"
  "cf6e6c41c08fa53df96254507e254422")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetHuskyCmdVel-request>)))
  "Returns full string definition for message of type '<SetHuskyCmdVel-request>"
  (cl:format cl:nil "geometry_msgs/Twist cmd_vel~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetHuskyCmdVel-request)))
  "Returns full string definition for message of type 'SetHuskyCmdVel-request"
  (cl:format cl:nil "geometry_msgs/Twist cmd_vel~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetHuskyCmdVel-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'cmd_vel))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetHuskyCmdVel-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetHuskyCmdVel-request
    (cl:cons ':cmd_vel (cmd_vel msg))
))
;//! \htmlinclude SetHuskyCmdVel-response.msg.html

(cl:defclass <SetHuskyCmdVel-response> (roslisp-msg-protocol:ros-message)
  ((success
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

(cl:defclass SetHuskyCmdVel-response (<SetHuskyCmdVel-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetHuskyCmdVel-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetHuskyCmdVel-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name husky_gazebo_plugins-srv:<SetHuskyCmdVel-response> is deprecated: use husky_gazebo_plugins-srv:SetHuskyCmdVel-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetHuskyCmdVel-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader husky_gazebo_plugins-srv:success-val is deprecated.  Use husky_gazebo_plugins-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'status_message-val :lambda-list '(m))
(cl:defmethod status_message-val ((m <SetHuskyCmdVel-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader husky_gazebo_plugins-srv:status_message-val is deprecated.  Use husky_gazebo_plugins-srv:status_message instead.")
  (status_message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetHuskyCmdVel-response>) ostream)
  "Serializes a message object of type '<SetHuskyCmdVel-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'status_message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'status_message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetHuskyCmdVel-response>) istream)
  "Deserializes a message object of type '<SetHuskyCmdVel-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetHuskyCmdVel-response>)))
  "Returns string type for a service object of type '<SetHuskyCmdVel-response>"
  "husky_gazebo_plugins/SetHuskyCmdVelResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetHuskyCmdVel-response)))
  "Returns string type for a service object of type 'SetHuskyCmdVel-response"
  "husky_gazebo_plugins/SetHuskyCmdVelResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetHuskyCmdVel-response>)))
  "Returns md5sum for a message object of type '<SetHuskyCmdVel-response>"
  "cf6e6c41c08fa53df96254507e254422")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetHuskyCmdVel-response)))
  "Returns md5sum for a message object of type 'SetHuskyCmdVel-response"
  "cf6e6c41c08fa53df96254507e254422")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetHuskyCmdVel-response>)))
  "Returns full string definition for message of type '<SetHuskyCmdVel-response>"
  (cl:format cl:nil "bool success                         # return true if set successful~%string status_message                # comments if available~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetHuskyCmdVel-response)))
  "Returns full string definition for message of type 'SetHuskyCmdVel-response"
  (cl:format cl:nil "bool success                         # return true if set successful~%string status_message                # comments if available~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetHuskyCmdVel-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'status_message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetHuskyCmdVel-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetHuskyCmdVel-response
    (cl:cons ':success (success msg))
    (cl:cons ':status_message (status_message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetHuskyCmdVel)))
  'SetHuskyCmdVel-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetHuskyCmdVel)))
  'SetHuskyCmdVel-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetHuskyCmdVel)))
  "Returns string type for a service object of type '<SetHuskyCmdVel>"
  "husky_gazebo_plugins/SetHuskyCmdVel")