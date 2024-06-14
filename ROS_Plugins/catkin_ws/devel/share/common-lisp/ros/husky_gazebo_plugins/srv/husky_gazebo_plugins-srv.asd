
(cl:in-package :asdf)

(defsystem "husky_gazebo_plugins-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :husky_gazebo_plugins-msg
               :nav_msgs-msg
               :sensor_msgs-msg
)
  :components ((:file "_package")
    (:file "GetHuskyJointStates" :depends-on ("_package_GetHuskyJointStates"))
    (:file "_package_GetHuskyJointStates" :depends-on ("_package"))
    (:file "GetHuskyOdometry" :depends-on ("_package_GetHuskyOdometry"))
    (:file "_package_GetHuskyOdometry" :depends-on ("_package"))
    (:file "SetHuskyCmdVel" :depends-on ("_package_SetHuskyCmdVel"))
    (:file "_package_SetHuskyCmdVel" :depends-on ("_package"))
    (:file "SetHuskyWheelSpeeds" :depends-on ("_package_SetHuskyWheelSpeeds"))
    (:file "_package_SetHuskyWheelSpeeds" :depends-on ("_package"))
  ))