; Auto-generated. Do not edit!


(cl:in-package navigation-msg)


;//! \htmlinclude img_result.msg.html

(cl:defclass <img_result> (roslisp-msg-protocol:ros-message)
  ((red
    :reader red
    :initarg :red
    :type cl:string
    :initform "")
   (shift
    :reader shift
    :initarg :shift
    :type cl:fixnum
    :initform 0))
)

(cl:defclass img_result (<img_result>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <img_result>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'img_result)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name navigation-msg:<img_result> is deprecated: use navigation-msg:img_result instead.")))

(cl:ensure-generic-function 'red-val :lambda-list '(m))
(cl:defmethod red-val ((m <img_result>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader navigation-msg:red-val is deprecated.  Use navigation-msg:red instead.")
  (red m))

(cl:ensure-generic-function 'shift-val :lambda-list '(m))
(cl:defmethod shift-val ((m <img_result>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader navigation-msg:shift-val is deprecated.  Use navigation-msg:shift instead.")
  (shift m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <img_result>) ostream)
  "Serializes a message object of type '<img_result>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'red))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'red))
  (cl:let* ((signed (cl:slot-value msg 'shift)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <img_result>) istream)
  "Deserializes a message object of type '<img_result>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'red) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'red) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'shift) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<img_result>)))
  "Returns string type for a message object of type '<img_result>"
  "navigation/img_result")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'img_result)))
  "Returns string type for a message object of type 'img_result"
  "navigation/img_result")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<img_result>)))
  "Returns md5sum for a message object of type '<img_result>"
  "d21b5d7806ece2676a2ff5c007553324")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'img_result)))
  "Returns md5sum for a message object of type 'img_result"
  "d21b5d7806ece2676a2ff5c007553324")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<img_result>)))
  "Returns full string definition for message of type '<img_result>"
  (cl:format cl:nil "string red ~%int16 shift~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'img_result)))
  "Returns full string definition for message of type 'img_result"
  (cl:format cl:nil "string red ~%int16 shift~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <img_result>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'red))
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <img_result>))
  "Converts a ROS message object to a list"
  (cl:list 'img_result
    (cl:cons ':red (red msg))
    (cl:cons ':shift (shift msg))
))
