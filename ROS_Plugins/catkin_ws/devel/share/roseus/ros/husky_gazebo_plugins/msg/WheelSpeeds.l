;; Auto-generated. Do not edit!


(when (boundp 'husky_gazebo_plugins::WheelSpeeds)
  (if (not (find-package "HUSKY_GAZEBO_PLUGINS"))
    (make-package "HUSKY_GAZEBO_PLUGINS"))
  (shadow 'WheelSpeeds (find-package "HUSKY_GAZEBO_PLUGINS")))
(unless (find-package "HUSKY_GAZEBO_PLUGINS::WHEELSPEEDS")
  (make-package "HUSKY_GAZEBO_PLUGINS::WHEELSPEEDS"))

(in-package "ROS")
;;//! \htmlinclude WheelSpeeds.msg.html


(defclass husky_gazebo_plugins::WheelSpeeds
  :super ros::object
  :slots (_back_left_wheel _back_right_wheel _front_left_wheel _front_right_wheel ))

(defmethod husky_gazebo_plugins::WheelSpeeds
  (:init
   (&key
    ((:back_left_wheel __back_left_wheel) 0.0)
    ((:back_right_wheel __back_right_wheel) 0.0)
    ((:front_left_wheel __front_left_wheel) 0.0)
    ((:front_right_wheel __front_right_wheel) 0.0)
    )
   (send-super :init)
   (setq _back_left_wheel (float __back_left_wheel))
   (setq _back_right_wheel (float __back_right_wheel))
   (setq _front_left_wheel (float __front_left_wheel))
   (setq _front_right_wheel (float __front_right_wheel))
   self)
  (:back_left_wheel
   (&optional __back_left_wheel)
   (if __back_left_wheel (setq _back_left_wheel __back_left_wheel)) _back_left_wheel)
  (:back_right_wheel
   (&optional __back_right_wheel)
   (if __back_right_wheel (setq _back_right_wheel __back_right_wheel)) _back_right_wheel)
  (:front_left_wheel
   (&optional __front_left_wheel)
   (if __front_left_wheel (setq _front_left_wheel __front_left_wheel)) _front_left_wheel)
  (:front_right_wheel
   (&optional __front_right_wheel)
   (if __front_right_wheel (setq _front_right_wheel __front_right_wheel)) _front_right_wheel)
  (:serialization-length
   ()
   (+
    ;; float64 _back_left_wheel
    8
    ;; float64 _back_right_wheel
    8
    ;; float64 _front_left_wheel
    8
    ;; float64 _front_right_wheel
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _back_left_wheel
       (sys::poke _back_left_wheel (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _back_right_wheel
       (sys::poke _back_right_wheel (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _front_left_wheel
       (sys::poke _front_left_wheel (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _front_right_wheel
       (sys::poke _front_right_wheel (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _back_left_wheel
     (setq _back_left_wheel (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _back_right_wheel
     (setq _back_right_wheel (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _front_left_wheel
     (setq _front_left_wheel (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _front_right_wheel
     (setq _front_right_wheel (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get husky_gazebo_plugins::WheelSpeeds :md5sum-) "6ce71b24a610adb7c8e0a82f915a151f")
(setf (get husky_gazebo_plugins::WheelSpeeds :datatype-) "husky_gazebo_plugins/WheelSpeeds")
(setf (get husky_gazebo_plugins::WheelSpeeds :definition-)
      "float64 back_left_wheel # back left wheel speed in m/s
float64 back_right_wheel # back right wheel speed in m/s
float64 front_left_wheel # front left wheel speed in m/s
float64 front_right_wheel # front right wheel speed in m/s

")



(provide :husky_gazebo_plugins/WheelSpeeds "6ce71b24a610adb7c8e0a82f915a151f")


