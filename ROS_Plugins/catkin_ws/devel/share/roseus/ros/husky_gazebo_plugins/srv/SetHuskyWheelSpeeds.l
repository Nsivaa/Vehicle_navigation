;; Auto-generated. Do not edit!


(when (boundp 'husky_gazebo_plugins::SetHuskyWheelSpeeds)
  (if (not (find-package "HUSKY_GAZEBO_PLUGINS"))
    (make-package "HUSKY_GAZEBO_PLUGINS"))
  (shadow 'SetHuskyWheelSpeeds (find-package "HUSKY_GAZEBO_PLUGINS")))
(unless (find-package "HUSKY_GAZEBO_PLUGINS::SETHUSKYWHEELSPEEDS")
  (make-package "HUSKY_GAZEBO_PLUGINS::SETHUSKYWHEELSPEEDS"))
(unless (find-package "HUSKY_GAZEBO_PLUGINS::SETHUSKYWHEELSPEEDSREQUEST")
  (make-package "HUSKY_GAZEBO_PLUGINS::SETHUSKYWHEELSPEEDSREQUEST"))
(unless (find-package "HUSKY_GAZEBO_PLUGINS::SETHUSKYWHEELSPEEDSRESPONSE")
  (make-package "HUSKY_GAZEBO_PLUGINS::SETHUSKYWHEELSPEEDSRESPONSE"))

(in-package "ROS")





(defclass husky_gazebo_plugins::SetHuskyWheelSpeedsRequest
  :super ros::object
  :slots (_wheel_speeds ))

(defmethod husky_gazebo_plugins::SetHuskyWheelSpeedsRequest
  (:init
   (&key
    ((:wheel_speeds __wheel_speeds) (instance husky_gazebo_plugins::WheelSpeeds :init))
    )
   (send-super :init)
   (setq _wheel_speeds __wheel_speeds)
   self)
  (:wheel_speeds
   (&rest __wheel_speeds)
   (if (keywordp (car __wheel_speeds))
       (send* _wheel_speeds __wheel_speeds)
     (progn
       (if __wheel_speeds (setq _wheel_speeds (car __wheel_speeds)))
       _wheel_speeds)))
  (:serialization-length
   ()
   (+
    ;; husky_gazebo_plugins/WheelSpeeds _wheel_speeds
    (send _wheel_speeds :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; husky_gazebo_plugins/WheelSpeeds _wheel_speeds
       (send _wheel_speeds :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; husky_gazebo_plugins/WheelSpeeds _wheel_speeds
     (send _wheel_speeds :deserialize buf ptr-) (incf ptr- (send _wheel_speeds :serialization-length))
   ;;
   self)
  )

(defclass husky_gazebo_plugins::SetHuskyWheelSpeedsResponse
  :super ros::object
  :slots (_success _status_message ))

(defmethod husky_gazebo_plugins::SetHuskyWheelSpeedsResponse
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

(defclass husky_gazebo_plugins::SetHuskyWheelSpeeds
  :super ros::object
  :slots ())

(setf (get husky_gazebo_plugins::SetHuskyWheelSpeeds :md5sum-) "ebe9a100c73aa02179b4e91ad134a89b")
(setf (get husky_gazebo_plugins::SetHuskyWheelSpeeds :datatype-) "husky_gazebo_plugins/SetHuskyWheelSpeeds")
(setf (get husky_gazebo_plugins::SetHuskyWheelSpeeds :request) husky_gazebo_plugins::SetHuskyWheelSpeedsRequest)
(setf (get husky_gazebo_plugins::SetHuskyWheelSpeeds :response) husky_gazebo_plugins::SetHuskyWheelSpeedsResponse)

(defmethod husky_gazebo_plugins::SetHuskyWheelSpeedsRequest
  (:response () (instance husky_gazebo_plugins::SetHuskyWheelSpeedsResponse :init)))

(setf (get husky_gazebo_plugins::SetHuskyWheelSpeedsRequest :md5sum-) "ebe9a100c73aa02179b4e91ad134a89b")
(setf (get husky_gazebo_plugins::SetHuskyWheelSpeedsRequest :datatype-) "husky_gazebo_plugins/SetHuskyWheelSpeedsRequest")
(setf (get husky_gazebo_plugins::SetHuskyWheelSpeedsRequest :definition-)
      "husky_gazebo_plugins/WheelSpeeds wheel_speeds

================================================================================
MSG: husky_gazebo_plugins/WheelSpeeds
float64 back_left_wheel # back left wheel speed in m/s
float64 back_right_wheel # back right wheel speed in m/s
float64 front_left_wheel # front left wheel speed in m/s
float64 front_right_wheel # front right wheel speed in m/s
---
bool success                         # return true if set successful
string status_message                # comments if available

")

(setf (get husky_gazebo_plugins::SetHuskyWheelSpeedsResponse :md5sum-) "ebe9a100c73aa02179b4e91ad134a89b")
(setf (get husky_gazebo_plugins::SetHuskyWheelSpeedsResponse :datatype-) "husky_gazebo_plugins/SetHuskyWheelSpeedsResponse")
(setf (get husky_gazebo_plugins::SetHuskyWheelSpeedsResponse :definition-)
      "husky_gazebo_plugins/WheelSpeeds wheel_speeds

================================================================================
MSG: husky_gazebo_plugins/WheelSpeeds
float64 back_left_wheel # back left wheel speed in m/s
float64 back_right_wheel # back right wheel speed in m/s
float64 front_left_wheel # front left wheel speed in m/s
float64 front_right_wheel # front right wheel speed in m/s
---
bool success                         # return true if set successful
string status_message                # comments if available

")



(provide :husky_gazebo_plugins/SetHuskyWheelSpeeds "ebe9a100c73aa02179b4e91ad134a89b")


