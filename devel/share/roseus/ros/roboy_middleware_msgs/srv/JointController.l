;; Auto-generated. Do not edit!


(when (boundp 'roboy_middleware_msgs::JointController)
  (if (not (find-package "ROBOY_MIDDLEWARE_MSGS"))
    (make-package "ROBOY_MIDDLEWARE_MSGS"))
  (shadow 'JointController (find-package "ROBOY_MIDDLEWARE_MSGS")))
(unless (find-package "ROBOY_MIDDLEWARE_MSGS::JOINTCONTROLLER")
  (make-package "ROBOY_MIDDLEWARE_MSGS::JOINTCONTROLLER"))
(unless (find-package "ROBOY_MIDDLEWARE_MSGS::JOINTCONTROLLERREQUEST")
  (make-package "ROBOY_MIDDLEWARE_MSGS::JOINTCONTROLLERREQUEST"))
(unless (find-package "ROBOY_MIDDLEWARE_MSGS::JOINTCONTROLLERRESPONSE")
  (make-package "ROBOY_MIDDLEWARE_MSGS::JOINTCONTROLLERRESPONSE"))

(in-package "ROS")





(defclass roboy_middleware_msgs::JointControllerRequest
  :super ros::object
  :slots (_elbow_control_enable _kp_elbow_joint _kd_elbow_joint _kp_elbow_agonist _kd_elbow_agonist _kp_elbow_antagonist _kd_elbow_antagonist _elbow_deadband _elbow_pretension _wrist_control_enable _kp_wrist_joint _kd_wrist_joint _kp_wrist_agonist _kd_wrist_agonist _kp_wrist_antagonist _kd_wrist_antagonist _wrist_deadband _wrist_pretension ))

(defmethod roboy_middleware_msgs::JointControllerRequest
  (:init
   (&key
    ((:elbow_control_enable __elbow_control_enable) nil)
    ((:kp_elbow_joint __kp_elbow_joint) 0)
    ((:kd_elbow_joint __kd_elbow_joint) 0)
    ((:kp_elbow_agonist __kp_elbow_agonist) 0)
    ((:kd_elbow_agonist __kd_elbow_agonist) 0)
    ((:kp_elbow_antagonist __kp_elbow_antagonist) 0)
    ((:kd_elbow_antagonist __kd_elbow_antagonist) 0)
    ((:elbow_deadband __elbow_deadband) 0)
    ((:elbow_pretension __elbow_pretension) 0)
    ((:wrist_control_enable __wrist_control_enable) nil)
    ((:kp_wrist_joint __kp_wrist_joint) 0)
    ((:kd_wrist_joint __kd_wrist_joint) 0)
    ((:kp_wrist_agonist __kp_wrist_agonist) 0)
    ((:kd_wrist_agonist __kd_wrist_agonist) 0)
    ((:kp_wrist_antagonist __kp_wrist_antagonist) 0)
    ((:kd_wrist_antagonist __kd_wrist_antagonist) 0)
    ((:wrist_deadband __wrist_deadband) 0)
    ((:wrist_pretension __wrist_pretension) 0)
    )
   (send-super :init)
   (setq _elbow_control_enable __elbow_control_enable)
   (setq _kp_elbow_joint (round __kp_elbow_joint))
   (setq _kd_elbow_joint (round __kd_elbow_joint))
   (setq _kp_elbow_agonist (round __kp_elbow_agonist))
   (setq _kd_elbow_agonist (round __kd_elbow_agonist))
   (setq _kp_elbow_antagonist (round __kp_elbow_antagonist))
   (setq _kd_elbow_antagonist (round __kd_elbow_antagonist))
   (setq _elbow_deadband (round __elbow_deadband))
   (setq _elbow_pretension (round __elbow_pretension))
   (setq _wrist_control_enable __wrist_control_enable)
   (setq _kp_wrist_joint (round __kp_wrist_joint))
   (setq _kd_wrist_joint (round __kd_wrist_joint))
   (setq _kp_wrist_agonist (round __kp_wrist_agonist))
   (setq _kd_wrist_agonist (round __kd_wrist_agonist))
   (setq _kp_wrist_antagonist (round __kp_wrist_antagonist))
   (setq _kd_wrist_antagonist (round __kd_wrist_antagonist))
   (setq _wrist_deadband (round __wrist_deadband))
   (setq _wrist_pretension (round __wrist_pretension))
   self)
  (:elbow_control_enable
   (&optional __elbow_control_enable)
   (if __elbow_control_enable (setq _elbow_control_enable __elbow_control_enable)) _elbow_control_enable)
  (:kp_elbow_joint
   (&optional __kp_elbow_joint)
   (if __kp_elbow_joint (setq _kp_elbow_joint __kp_elbow_joint)) _kp_elbow_joint)
  (:kd_elbow_joint
   (&optional __kd_elbow_joint)
   (if __kd_elbow_joint (setq _kd_elbow_joint __kd_elbow_joint)) _kd_elbow_joint)
  (:kp_elbow_agonist
   (&optional __kp_elbow_agonist)
   (if __kp_elbow_agonist (setq _kp_elbow_agonist __kp_elbow_agonist)) _kp_elbow_agonist)
  (:kd_elbow_agonist
   (&optional __kd_elbow_agonist)
   (if __kd_elbow_agonist (setq _kd_elbow_agonist __kd_elbow_agonist)) _kd_elbow_agonist)
  (:kp_elbow_antagonist
   (&optional __kp_elbow_antagonist)
   (if __kp_elbow_antagonist (setq _kp_elbow_antagonist __kp_elbow_antagonist)) _kp_elbow_antagonist)
  (:kd_elbow_antagonist
   (&optional __kd_elbow_antagonist)
   (if __kd_elbow_antagonist (setq _kd_elbow_antagonist __kd_elbow_antagonist)) _kd_elbow_antagonist)
  (:elbow_deadband
   (&optional __elbow_deadband)
   (if __elbow_deadband (setq _elbow_deadband __elbow_deadband)) _elbow_deadband)
  (:elbow_pretension
   (&optional __elbow_pretension)
   (if __elbow_pretension (setq _elbow_pretension __elbow_pretension)) _elbow_pretension)
  (:wrist_control_enable
   (&optional __wrist_control_enable)
   (if __wrist_control_enable (setq _wrist_control_enable __wrist_control_enable)) _wrist_control_enable)
  (:kp_wrist_joint
   (&optional __kp_wrist_joint)
   (if __kp_wrist_joint (setq _kp_wrist_joint __kp_wrist_joint)) _kp_wrist_joint)
  (:kd_wrist_joint
   (&optional __kd_wrist_joint)
   (if __kd_wrist_joint (setq _kd_wrist_joint __kd_wrist_joint)) _kd_wrist_joint)
  (:kp_wrist_agonist
   (&optional __kp_wrist_agonist)
   (if __kp_wrist_agonist (setq _kp_wrist_agonist __kp_wrist_agonist)) _kp_wrist_agonist)
  (:kd_wrist_agonist
   (&optional __kd_wrist_agonist)
   (if __kd_wrist_agonist (setq _kd_wrist_agonist __kd_wrist_agonist)) _kd_wrist_agonist)
  (:kp_wrist_antagonist
   (&optional __kp_wrist_antagonist)
   (if __kp_wrist_antagonist (setq _kp_wrist_antagonist __kp_wrist_antagonist)) _kp_wrist_antagonist)
  (:kd_wrist_antagonist
   (&optional __kd_wrist_antagonist)
   (if __kd_wrist_antagonist (setq _kd_wrist_antagonist __kd_wrist_antagonist)) _kd_wrist_antagonist)
  (:wrist_deadband
   (&optional __wrist_deadband)
   (if __wrist_deadband (setq _wrist_deadband __wrist_deadband)) _wrist_deadband)
  (:wrist_pretension
   (&optional __wrist_pretension)
   (if __wrist_pretension (setq _wrist_pretension __wrist_pretension)) _wrist_pretension)
  (:serialization-length
   ()
   (+
    ;; bool _elbow_control_enable
    1
    ;; int32 _kp_elbow_joint
    4
    ;; int32 _kd_elbow_joint
    4
    ;; int32 _kp_elbow_agonist
    4
    ;; int32 _kd_elbow_agonist
    4
    ;; int32 _kp_elbow_antagonist
    4
    ;; int32 _kd_elbow_antagonist
    4
    ;; int32 _elbow_deadband
    4
    ;; int32 _elbow_pretension
    4
    ;; bool _wrist_control_enable
    1
    ;; int32 _kp_wrist_joint
    4
    ;; int32 _kd_wrist_joint
    4
    ;; int32 _kp_wrist_agonist
    4
    ;; int32 _kd_wrist_agonist
    4
    ;; int32 _kp_wrist_antagonist
    4
    ;; int32 _kd_wrist_antagonist
    4
    ;; int32 _wrist_deadband
    4
    ;; int32 _wrist_pretension
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _elbow_control_enable
       (if _elbow_control_enable (write-byte -1 s) (write-byte 0 s))
     ;; int32 _kp_elbow_joint
       (write-long _kp_elbow_joint s)
     ;; int32 _kd_elbow_joint
       (write-long _kd_elbow_joint s)
     ;; int32 _kp_elbow_agonist
       (write-long _kp_elbow_agonist s)
     ;; int32 _kd_elbow_agonist
       (write-long _kd_elbow_agonist s)
     ;; int32 _kp_elbow_antagonist
       (write-long _kp_elbow_antagonist s)
     ;; int32 _kd_elbow_antagonist
       (write-long _kd_elbow_antagonist s)
     ;; int32 _elbow_deadband
       (write-long _elbow_deadband s)
     ;; int32 _elbow_pretension
       (write-long _elbow_pretension s)
     ;; bool _wrist_control_enable
       (if _wrist_control_enable (write-byte -1 s) (write-byte 0 s))
     ;; int32 _kp_wrist_joint
       (write-long _kp_wrist_joint s)
     ;; int32 _kd_wrist_joint
       (write-long _kd_wrist_joint s)
     ;; int32 _kp_wrist_agonist
       (write-long _kp_wrist_agonist s)
     ;; int32 _kd_wrist_agonist
       (write-long _kd_wrist_agonist s)
     ;; int32 _kp_wrist_antagonist
       (write-long _kp_wrist_antagonist s)
     ;; int32 _kd_wrist_antagonist
       (write-long _kd_wrist_antagonist s)
     ;; int32 _wrist_deadband
       (write-long _wrist_deadband s)
     ;; int32 _wrist_pretension
       (write-long _wrist_pretension s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _elbow_control_enable
     (setq _elbow_control_enable (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; int32 _kp_elbow_joint
     (setq _kp_elbow_joint (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _kd_elbow_joint
     (setq _kd_elbow_joint (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _kp_elbow_agonist
     (setq _kp_elbow_agonist (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _kd_elbow_agonist
     (setq _kd_elbow_agonist (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _kp_elbow_antagonist
     (setq _kp_elbow_antagonist (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _kd_elbow_antagonist
     (setq _kd_elbow_antagonist (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _elbow_deadband
     (setq _elbow_deadband (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _elbow_pretension
     (setq _elbow_pretension (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool _wrist_control_enable
     (setq _wrist_control_enable (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; int32 _kp_wrist_joint
     (setq _kp_wrist_joint (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _kd_wrist_joint
     (setq _kd_wrist_joint (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _kp_wrist_agonist
     (setq _kp_wrist_agonist (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _kd_wrist_agonist
     (setq _kd_wrist_agonist (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _kp_wrist_antagonist
     (setq _kp_wrist_antagonist (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _kd_wrist_antagonist
     (setq _kd_wrist_antagonist (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _wrist_deadband
     (setq _wrist_deadband (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _wrist_pretension
     (setq _wrist_pretension (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass roboy_middleware_msgs::JointControllerResponse
  :super ros::object
  :slots ())

(defmethod roboy_middleware_msgs::JointControllerResponse
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

(defclass roboy_middleware_msgs::JointController
  :super ros::object
  :slots ())

(setf (get roboy_middleware_msgs::JointController :md5sum-) "8c2b216fbd74af3eb65d1ca2f72cefc3")
(setf (get roboy_middleware_msgs::JointController :datatype-) "roboy_middleware_msgs/JointController")
(setf (get roboy_middleware_msgs::JointController :request) roboy_middleware_msgs::JointControllerRequest)
(setf (get roboy_middleware_msgs::JointController :response) roboy_middleware_msgs::JointControllerResponse)

(defmethod roboy_middleware_msgs::JointControllerRequest
  (:response () (instance roboy_middleware_msgs::JointControllerResponse :init)))

(setf (get roboy_middleware_msgs::JointControllerRequest :md5sum-) "8c2b216fbd74af3eb65d1ca2f72cefc3")
(setf (get roboy_middleware_msgs::JointControllerRequest :datatype-) "roboy_middleware_msgs/JointControllerRequest")
(setf (get roboy_middleware_msgs::JointControllerRequest :definition-)
      "bool elbow_control_enable
int32 kp_elbow_joint
int32 kd_elbow_joint
int32 kp_elbow_agonist
int32 kd_elbow_agonist
int32 kp_elbow_antagonist
int32 kd_elbow_antagonist
int32 elbow_deadband
int32 elbow_pretension
bool wrist_control_enable
int32 kp_wrist_joint
int32 kd_wrist_joint
int32 kp_wrist_agonist
int32 kd_wrist_agonist
int32 kp_wrist_antagonist
int32 kd_wrist_antagonist
int32 wrist_deadband
int32 wrist_pretension
---

")

(setf (get roboy_middleware_msgs::JointControllerResponse :md5sum-) "8c2b216fbd74af3eb65d1ca2f72cefc3")
(setf (get roboy_middleware_msgs::JointControllerResponse :datatype-) "roboy_middleware_msgs/JointControllerResponse")
(setf (get roboy_middleware_msgs::JointControllerResponse :definition-)
      "bool elbow_control_enable
int32 kp_elbow_joint
int32 kd_elbow_joint
int32 kp_elbow_agonist
int32 kd_elbow_agonist
int32 kp_elbow_antagonist
int32 kd_elbow_antagonist
int32 elbow_deadband
int32 elbow_pretension
bool wrist_control_enable
int32 kp_wrist_joint
int32 kd_wrist_joint
int32 kp_wrist_agonist
int32 kd_wrist_agonist
int32 kp_wrist_antagonist
int32 kd_wrist_antagonist
int32 wrist_deadband
int32 wrist_pretension
---

")



(provide :roboy_middleware_msgs/JointController "8c2b216fbd74af3eb65d1ca2f72cefc3")


