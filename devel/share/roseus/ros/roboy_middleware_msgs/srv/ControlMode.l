;; Auto-generated. Do not edit!


(when (boundp 'roboy_middleware_msgs::ControlMode)
  (if (not (find-package "ROBOY_MIDDLEWARE_MSGS"))
    (make-package "ROBOY_MIDDLEWARE_MSGS"))
  (shadow 'ControlMode (find-package "ROBOY_MIDDLEWARE_MSGS")))
(unless (find-package "ROBOY_MIDDLEWARE_MSGS::CONTROLMODE")
  (make-package "ROBOY_MIDDLEWARE_MSGS::CONTROLMODE"))
(unless (find-package "ROBOY_MIDDLEWARE_MSGS::CONTROLMODEREQUEST")
  (make-package "ROBOY_MIDDLEWARE_MSGS::CONTROLMODEREQUEST"))
(unless (find-package "ROBOY_MIDDLEWARE_MSGS::CONTROLMODERESPONSE")
  (make-package "ROBOY_MIDDLEWARE_MSGS::CONTROLMODERESPONSE"))

(in-package "ROS")





(defclass roboy_middleware_msgs::ControlModeRequest
  :super ros::object
  :slots (_control_mode _set_point _motor_id ))

(defmethod roboy_middleware_msgs::ControlModeRequest
  (:init
   (&key
    ((:control_mode __control_mode) 0)
    ((:set_point __set_point) 0)
    ((:motor_id __motor_id) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _control_mode (round __control_mode))
   (setq _set_point (round __set_point))
   (setq _motor_id __motor_id)
   self)
  (:control_mode
   (&optional __control_mode)
   (if __control_mode (setq _control_mode __control_mode)) _control_mode)
  (:set_point
   (&optional __set_point)
   (if __set_point (setq _set_point __set_point)) _set_point)
  (:motor_id
   (&optional __motor_id)
   (if __motor_id (setq _motor_id __motor_id)) _motor_id)
  (:serialization-length
   ()
   (+
    ;; int32 _control_mode
    4
    ;; int32 _set_point
    4
    ;; uint16[] _motor_id
    (* 2    (length _motor_id)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _control_mode
       (write-long _control_mode s)
     ;; int32 _set_point
       (write-long _set_point s)
     ;; uint16[] _motor_id
     (write-long (length _motor_id) s)
     (dotimes (i (length _motor_id))
       (write-word (elt _motor_id i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _control_mode
     (setq _control_mode (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _set_point
     (setq _set_point (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint16[] _motor_id
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _motor_id (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _motor_id i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     ))
   ;;
   self)
  )

(defclass roboy_middleware_msgs::ControlModeResponse
  :super ros::object
  :slots ())

(defmethod roboy_middleware_msgs::ControlModeResponse
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass roboy_middleware_msgs::ControlMode
  :super ros::object
  :slots ())

(setf (get roboy_middleware_msgs::ControlMode :md5sum-) "3bf3f3c500bf7df65355716be531bee1")
(setf (get roboy_middleware_msgs::ControlMode :datatype-) "roboy_middleware_msgs/ControlMode")
(setf (get roboy_middleware_msgs::ControlMode :request) roboy_middleware_msgs::ControlModeRequest)
(setf (get roboy_middleware_msgs::ControlMode :response) roboy_middleware_msgs::ControlModeResponse)

(defmethod roboy_middleware_msgs::ControlModeRequest
  (:response () (instance roboy_middleware_msgs::ControlModeResponse :init)))

(setf (get roboy_middleware_msgs::ControlModeRequest :md5sum-) "3bf3f3c500bf7df65355716be531bee1")
(setf (get roboy_middleware_msgs::ControlModeRequest :datatype-) "roboy_middleware_msgs/ControlModeRequest")
(setf (get roboy_middleware_msgs::ControlModeRequest :definition-)
      "int32 control_mode
int32 set_point

uint16[] motor_id
---

")

(setf (get roboy_middleware_msgs::ControlModeResponse :md5sum-) "3bf3f3c500bf7df65355716be531bee1")
(setf (get roboy_middleware_msgs::ControlModeResponse :datatype-) "roboy_middleware_msgs/ControlModeResponse")
(setf (get roboy_middleware_msgs::ControlModeResponse :definition-)
      "int32 control_mode
int32 set_point

uint16[] motor_id
---

")



(provide :roboy_middleware_msgs/ControlMode "3bf3f3c500bf7df65355716be531bee1")


