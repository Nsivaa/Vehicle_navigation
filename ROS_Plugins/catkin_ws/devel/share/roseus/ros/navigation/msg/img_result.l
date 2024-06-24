;; Auto-generated. Do not edit!


(when (boundp 'navigation::img_result)
  (if (not (find-package "NAVIGATION"))
    (make-package "NAVIGATION"))
  (shadow 'img_result (find-package "NAVIGATION")))
(unless (find-package "NAVIGATION::IMG_RESULT")
  (make-package "NAVIGATION::IMG_RESULT"))

(in-package "ROS")
;;//! \htmlinclude img_result.msg.html


(defclass navigation::img_result
  :super ros::object
  :slots (_red _shift ))

(defmethod navigation::img_result
  (:init
   (&key
    ((:red __red) "")
    ((:shift __shift) 0)
    )
   (send-super :init)
   (setq _red (string __red))
   (setq _shift (round __shift))
   self)
  (:red
   (&optional __red)
   (if __red (setq _red __red)) _red)
  (:shift
   (&optional __shift)
   (if __shift (setq _shift __shift)) _shift)
  (:serialization-length
   ()
   (+
    ;; string _red
    4 (length _red)
    ;; int16 _shift
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _red
       (write-long (length _red) s) (princ _red s)
     ;; int16 _shift
       (write-word _shift s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _red
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _red (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int16 _shift
     (setq _shift (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get navigation::img_result :md5sum-) "d21b5d7806ece2676a2ff5c007553324")
(setf (get navigation::img_result :datatype-) "navigation/img_result")
(setf (get navigation::img_result :definition-)
      "string red 
int16 shift


")



(provide :navigation/img_result "d21b5d7806ece2676a2ff5c007553324")


