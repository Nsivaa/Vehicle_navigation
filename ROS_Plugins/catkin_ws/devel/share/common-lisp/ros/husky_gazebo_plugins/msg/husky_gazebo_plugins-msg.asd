
(cl:in-package :asdf)

(defsystem "husky_gazebo_plugins-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "WheelSpeeds" :depends-on ("_package_WheelSpeeds"))
    (:file "_package_WheelSpeeds" :depends-on ("_package"))
  ))