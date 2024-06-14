; Auto-generated. Do not edit!


(cl:in-package husky_gazebo_plugins-srv)


;//! \htmlinclude SetHuskyWheelSpeeds-request.msg.html

(cl:defclass <SetHuskyWheelSpeeds-request> (roslisp-msg-protocol:ros-message)
  ((wheel_speeds
    :reader wheel_speeds
    :initarg :wheel_speeds
    :type husky_gazebo_plugins-msg:WheelSpeeds
    :initform (cl:make-instance 'husky_gazebo_plugins-msg:WheelSpeeds)))
)

(cl:defclass SetHuskyWheelSpeeds-request (<SetHuskyWheelSpeeds-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetHuskyWheelSpeeds-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetHuskyWheelSpeeds-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name husky_gazebo_plugins-srv:<SetHuskyWheelSpeeds-request> is deprecated: use husky_gazebo_plugins-srv:SetHuskyWheelSpeeds-request instead.")))

(cl:ensure-generic-function 'wheel_speeds-val :lambda-list '(m))
(cl:defmethod wheel_speeds-val ((m <SetHuskyWheelSpeeds-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader husky_gazebo_plugins-srv:wheel_speeds-val is deprecated.  Use husky_gazebo_plugins-srv:wheel_speeds instead.")
  (wheel_speeds m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetHuskyWheelSpeeds-request>) ostream)
  "Serializes a message object of type '<SetHuskyWheelSpeeds-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'wheel_speeds) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetHuskyWheelSpeeds-request>) istream)
  "Deserializes a message object of type '<SetHuskyWheelSpeeds-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'wheel_speeds) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetHuskyWheelSpeeds-request>)))
  "Returns string type for a service object of type '<SetHuskyWheelSpeeds-request>"
  "husky_gazebo_plugins/SetHuskyWheelSpeedsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetHuskyWheelSpeeds-request)))
  "Returns string type for a service object of type 'SetHuskyWheelSpeeds-request"
  "husky_gazebo_plugins/SetHuskyWheelSpeedsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetHuskyWheelSpeeds-request>)))
  "Returns md5sum for a message object of type '<SetHuskyWheelSpeeds-request>"
  "ebe9a100c73aa02179b4e91ad134a89b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetHuskyWheelSpeeds-request)))
  "Returns md5sum for a message object of type 'SetHuskyWheelSpeeds-request"
  "ebe9a100c73aa02179b4e91ad134a89b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetHuskyWheelSpeeds-request>)))
  "Returns full string definition for message of type '<SetHuskyWheelSpeeds-request>"
  (cl:format cl:nil "husky_gazebo_plugins/WheelSpeeds wheel_speeds~%~%================================================================================~%MSG: husky_gazebo_plugins/WheelSpeeds~%float64 back_left_wheel # back left wheel speed in m/s~%float64 back_right_wheel # back right wheel speed in m/s~%float64 front_left_wheel # front left wheel speed in m/s~%float64 front_right_wheel # front right wheel speed in m/s~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetHuskyWheelSpeeds-request)))
  "Returns full string definition for message of type 'SetHuskyWheelSpeeds-request"
  (cl:format cl:nil "husky_gazebo_plugins/WheelSpeeds wheel_speeds~%~%================================================================================~%MSG: husky_gazebo_plugins/WheelSpeeds~%float64 back_left_wheel # back left wheel speed in m/s~%float64 back_right_wheel # back right wheel speed in m/s~%float64 front_left_wheel # front left wheel speed in m/s~%float64 front_right_wheel # front right wheel speed in m/s~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetHuskyWheelSpeeds-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'wheel_speeds))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetHuskyWheelSpeeds-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetHuskyWheelSpeeds-request
    (cl:cons ':wheel_speeds (wheel_speeds msg))
))
;//! \htmlinclude SetHuskyWheelSpeeds-response.msg.html

(cl:defclass <SetHuskyWheelSpeeds-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass SetHuskyWheelSpeeds-response (<SetHuskyWheelSpeeds-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetHuskyWheelSpeeds-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetHuskyWheelSpeeds-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name husky_gazebo_plugins-srv:<SetHuskyWheelSpeeds-response> is deprecated: use husky_gazebo_plugins-srv:SetHuskyWheelSpeeds-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetHuskyWheelSpeeds-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader husky_gazebo_plugins-srv:success-val is deprecated.  Use husky_gazebo_plugins-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'status_message-val :lambda-list '(m))
(cl:defmethod status_message-val ((m <SetHuskyWheelSpeeds-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader husky_gazebo_plugins-srv:status_message-val is deprecated.  Use husky_gazebo_plugins-srv:status_message instead.")
  (status_message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetHuskyWheelSpeeds-response>) ostream)
  "Serializes a message object of type '<SetHuskyWheelSpeeds-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'status_message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'status_message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetHuskyWheelSpeeds-response>) istream)
  "Deserializes a message object of type '<SetHuskyWheelSpeeds-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetHuskyWheelSpeeds-response>)))
  "Returns string type for a service object of type '<SetHuskyWheelSpeeds-response>"
  "husky_gazebo_plugins/SetHuskyWheelSpeedsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetHuskyWheelSpeeds-response)))
  "Returns string type for a service object of type 'SetHuskyWheelSpeeds-response"
  "husky_gazebo_plugins/SetHuskyWheelSpeedsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetHuskyWheelSpeeds-response>)))
  "Returns md5sum for a message object of type '<SetHuskyWheelSpeeds-response>"
  "ebe9a100c73aa02179b4e91ad134a89b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetHuskyWheelSpeeds-response)))
  "Returns md5sum for a message object of type 'SetHuskyWheelSpeeds-response"
  "ebe9a100c73aa02179b4e91ad134a89b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetHuskyWheelSpeeds-response>)))
  "Returns full string definition for message of type '<SetHuskyWheelSpeeds-response>"
  (cl:format cl:nil "bool success                         # return true if set successful~%string status_message                # comments if available~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetHuskyWheelSpeeds-response)))
  "Returns full string definition for message of type 'SetHuskyWheelSpeeds-response"
  (cl:format cl:nil "bool success                         # return true if set successful~%string status_message                # comments if available~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetHuskyWheelSpeeds-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'status_message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetHuskyWheelSpeeds-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetHuskyWheelSpeeds-response
    (cl:cons ':success (success msg))
    (cl:cons ':status_message (status_message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetHuskyWheelSpeeds)))
  'SetHuskyWheelSpeeds-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetHuskyWheelSpeeds)))
  'SetHuskyWheelSpeeds-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetHuskyWheelSpeeds)))
  "Returns string type for a service object of type '<SetHuskyWheelSpeeds>"
  "husky_gazebo_plugins/SetHuskyWheelSpeeds")