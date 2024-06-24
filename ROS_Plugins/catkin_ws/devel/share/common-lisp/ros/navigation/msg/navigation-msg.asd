
(cl:in-package :asdf)

(defsystem "navigation-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "img_result" :depends-on ("_package_img_result"))
    (:file "_package_img_result" :depends-on ("_package"))
  ))