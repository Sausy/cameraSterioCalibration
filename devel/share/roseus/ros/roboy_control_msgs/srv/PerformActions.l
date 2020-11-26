;; Auto-generated. Do not edit!


(when (boundp 'roboy_control_msgs::PerformActions)
  (if (not (find-package "ROBOY_CONTROL_MSGS"))
    (make-package "ROBOY_CONTROL_MSGS"))
  (shadow 'PerformActions (find-package "ROBOY_CONTROL_MSGS")))
(unless (find-package "ROBOY_CONTROL_MSGS::PERFORMACTIONS")
  (make-package "ROBOY_CONTROL_MSGS::PERFORMACTIONS"))
(unless (find-package "ROBOY_CONTROL_MSGS::PERFORMACTIONSREQUEST")
  (make-package "ROBOY_CONTROL_MSGS::PERFORMACTIONSREQUEST"))
(unless (find-package "ROBOY_CONTROL_MSGS::PERFORMACTIONSRESPONSE")
  (make-package "ROBOY_CONTROL_MSGS::PERFORMACTIONSRESPONSE"))

(in-package "ROS")





(defclass roboy_control_msgs::PerformActionsRequest
  :super ros::object
  :slots (_actions ))

(defmethod roboy_control_msgs::PerformActionsRequest
  (:init
   (&key
    ((:actions __actions) (let (r) (dotimes (i 0) (push "" r)) r))
    )
   (send-super :init)
   (setq _actions __actions)
   self)
  (:actions
   (&optional __actions)
   (if __actions (setq _actions __actions)) _actions)
  (:serialization-length
   ()
   (+
    ;; string[] _actions
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _actions)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string[] _actions
     (write-long (length _actions) s)
     (dolist (elem _actions)
       (write-long (length elem) s) (princ elem s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string[] _actions
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _actions (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _actions i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;;
   self)
  )

(defclass roboy_control_msgs::PerformActionsResponse
  :super ros::object
  :slots (_success ))

(defmethod roboy_control_msgs::PerformActionsResponse
  (:init
   (&key
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _success __success)
   self)
  (:success
   (&optional __success)
   (if __success (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass roboy_control_msgs::PerformActions
  :super ros::object
  :slots ())

(setf (get roboy_control_msgs::PerformActions :md5sum-) "4c7ae25d13777401a5c311c271b76cf4")
(setf (get roboy_control_msgs::PerformActions :datatype-) "roboy_control_msgs/PerformActions")
(setf (get roboy_control_msgs::PerformActions :request) roboy_control_msgs::PerformActionsRequest)
(setf (get roboy_control_msgs::PerformActions :response) roboy_control_msgs::PerformActionsResponse)

(defmethod roboy_control_msgs::PerformActionsRequest
  (:response () (instance roboy_control_msgs::PerformActionsResponse :init)))

(setf (get roboy_control_msgs::PerformActionsRequest :md5sum-) "4c7ae25d13777401a5c311c271b76cf4")
(setf (get roboy_control_msgs::PerformActionsRequest :datatype-) "roboy_control_msgs/PerformActionsRequest")
(setf (get roboy_control_msgs::PerformActionsRequest :definition-)
      "string[] actions # can be a trajectory name or pause (2_pause will pause for 2 seconds)
---
bool success
")

(setf (get roboy_control_msgs::PerformActionsResponse :md5sum-) "4c7ae25d13777401a5c311c271b76cf4")
(setf (get roboy_control_msgs::PerformActionsResponse :datatype-) "roboy_control_msgs/PerformActionsResponse")
(setf (get roboy_control_msgs::PerformActionsResponse :definition-)
      "string[] actions # can be a trajectory name or pause (2_pause will pause for 2 seconds)
---
bool success
")



(provide :roboy_control_msgs/PerformActions "4c7ae25d13777401a5c311c271b76cf4")


