;; Auto-generated. Do not edit!


(when (boundp 'husky_gazebo_plugins::SetHuskyCmdVel)
  (if (not (find-package "HUSKY_GAZEBO_PLUGINS"))
    (make-package "HUSKY_GAZEBO_PLUGINS"))
  (shadow 'SetHuskyCmdVel (find-package "HUSKY_GAZEBO_PLUGINS")))
(unless (find-package "HUSKY_GAZEBO_PLUGINS::SETHUSKYCMDVEL")
  (make-package "HUSKY_GAZEBO_PLUGINS::SETHUSKYCMDVEL"))
(unless (find-package "HUSKY_GAZEBO_PLUGINS::SETHUSKYCMDVELREQUEST")
  (make-package "HUSKY_GAZEBO_PLUGINS::SETHUSKYCMDVELREQUEST"))
(unless (find-package "HUSKY_GAZEBO_PLUGINS::SETHUSKYCMDVELRESPONSE")
  (make-package "HUSKY_GAZEBO_PLUGINS::SETHUSKYCMDVELRESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))




(defclass husky_gazebo_plugins::SetHuskyCmdVelRequest
  :super ros::object
  :slots (_cmd_vel ))

(defmethod husky_gazebo_plugins::SetHuskyCmdVelRequest
  (:init
   (&key
    ((:cmd_vel __cmd_vel) (instance geometry_msgs::Twist :init))
    )
   (send-super :init)
   (setq _cmd_vel __cmd_vel)
   self)
  (:cmd_vel
   (&rest __cmd_vel)
   (if (keywordp (car __cmd_vel))
       (send* _cmd_vel __cmd_vel)
     (progn
       (if __cmd_vel (setq _cmd_vel (car __cmd_vel)))
       _cmd_vel)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Twist _cmd_vel
    (send _cmd_vel :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Twist _cmd_vel
       (send _cmd_vel :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Twist _cmd_vel
     (send _cmd_vel :deserialize buf ptr-) (incf ptr- (send _cmd_vel :serialization-length))
   ;;
   self)
  )

(defclass husky_gazebo_plugins::SetHuskyCmdVelResponse
  :super ros::object
  :slots (_success _status_message ))

(defmethod husky_gazebo_plugins::SetHuskyCmdVelResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:status_message __status_message) "")
    )
   (send-super :init)
   (setq _success __success)
   (setq _status_message (string __status_message))
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:status_message
   (&optional __status_message)
   (if __status_message (setq _status_message __status_message)) _status_message)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; string _status_message
    4 (length _status_message)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _status_message
       (write-long (length _status_message) s) (princ _status_message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _status_message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _status_message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass husky_gazebo_plugins::SetHuskyCmdVel
  :super ros::object
  :slots ())

(setf (get husky_gazebo_plugins::SetHuskyCmdVel :md5sum-) "cf6e6c41c08fa53df96254507e254422")
(setf (get husky_gazebo_plugins::SetHuskyCmdVel :datatype-) "husky_gazebo_plugins/SetHuskyCmdVel")
(setf (get husky_gazebo_plugins::SetHuskyCmdVel :request) husky_gazebo_plugins::SetHuskyCmdVelRequest)
(setf (get husky_gazebo_plugins::SetHuskyCmdVel :response) husky_gazebo_plugins::SetHuskyCmdVelResponse)

(defmethod husky_gazebo_plugins::SetHuskyCmdVelRequest
  (:response () (instance husky_gazebo_plugins::SetHuskyCmdVelResponse :init)))

(setf (get husky_gazebo_plugins::SetHuskyCmdVelRequest :md5sum-) "cf6e6c41c08fa53df96254507e254422")
(setf (get husky_gazebo_plugins::SetHuskyCmdVelRequest :datatype-) "husky_gazebo_plugins/SetHuskyCmdVelRequest")
(setf (get husky_gazebo_plugins::SetHuskyCmdVelRequest :definition-)
      "geometry_msgs/Twist cmd_vel

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
---
bool success                         # return true if set successful
string status_message                # comments if available

")

(setf (get husky_gazebo_plugins::SetHuskyCmdVelResponse :md5sum-) "cf6e6c41c08fa53df96254507e254422")
(setf (get husky_gazebo_plugins::SetHuskyCmdVelResponse :datatype-) "husky_gazebo_plugins/SetHuskyCmdVelResponse")
(setf (get husky_gazebo_plugins::SetHuskyCmdVelResponse :definition-)
      "geometry_msgs/Twist cmd_vel

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
---
bool success                         # return true if set successful
string status_message                # comments if available

")



(provide :husky_gazebo_plugins/SetHuskyCmdVel "cf6e6c41c08fa53df96254507e254422")


