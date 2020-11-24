;; Auto-generated. Do not edit!


(when (boundp 'roboy_middleware_msgs::MotorConfigService)
  (if (not (find-package "ROBOY_MIDDLEWARE_MSGS"))
    (make-package "ROBOY_MIDDLEWARE_MSGS"))
  (shadow 'MotorConfigService (find-package "ROBOY_MIDDLEWARE_MSGS")))
(unless (find-package "ROBOY_MIDDLEWARE_MSGS::MOTORCONFIGSERVICE")
  (make-package "ROBOY_MIDDLEWARE_MSGS::MOTORCONFIGSERVICE"))
(unless (find-package "ROBOY_MIDDLEWARE_MSGS::MOTORCONFIGSERVICEREQUEST")
  (make-package "ROBOY_MIDDLEWARE_MSGS::MOTORCONFIGSERVICEREQUEST"))
(unless (find-package "ROBOY_MIDDLEWARE_MSGS::MOTORCONFIGSERVICERESPONSE")
  (make-package "ROBOY_MIDDLEWARE_MSGS::MOTORCONFIGSERVICERESPONSE"))

(in-package "ROS")





(defclass roboy_middleware_msgs::MotorConfigServiceRequest
  :super ros::object
  :slots (_config ))

(defmethod roboy_middleware_msgs::MotorConfigServiceRequest
  (:init
   (&key
    ((:config __config) (instance roboy_middleware_msgs::MotorConfig :init))
    )
   (send-super :init)
   (setq _config __config)
   self)
  (:config
   (&rest __config)
   (if (keywordp (car __config))
       (send* _config __config)
     (progn
       (if __config (setq _config (car __config)))
       _config)))
  (:serialization-length
   ()
   (+
    ;; roboy_middleware_msgs/MotorConfig _config
    (send _config :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; roboy_middleware_msgs/MotorConfig _config
       (send _config :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; roboy_middleware_msgs/MotorConfig _config
     (send _config :deserialize buf ptr-) (incf ptr- (send _config :serialization-length))
   ;;
   self)
  )

(defclass roboy_middleware_msgs::MotorConfigServiceResponse
  :super ros::object
  :slots (_mode ))

(defmethod roboy_middleware_msgs::MotorConfigServiceResponse
  (:init
   (&key
    ((:mode __mode) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _mode __mode)
   self)
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:serialization-length
   ()
   (+
    ;; int32[] _mode
    (* 4    (length _mode)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32[] _mode
     (write-long (length _mode) s)
     (dotimes (i (length _mode))
       (write-long (elt _mode i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32[] _mode
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _mode (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _mode i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(defclass roboy_middleware_msgs::MotorConfigService
  :super ros::object
  :slots ())

(setf (get roboy_middleware_msgs::MotorConfigService :md5sum-) "0aa4adc3b3f52f02aacbfa621182887c")
(setf (get roboy_middleware_msgs::MotorConfigService :datatype-) "roboy_middleware_msgs/MotorConfigService")
(setf (get roboy_middleware_msgs::MotorConfigService :request) roboy_middleware_msgs::MotorConfigServiceRequest)
(setf (get roboy_middleware_msgs::MotorConfigService :response) roboy_middleware_msgs::MotorConfigServiceResponse)

(defmethod roboy_middleware_msgs::MotorConfigServiceRequest
  (:response () (instance roboy_middleware_msgs::MotorConfigServiceResponse :init)))

(setf (get roboy_middleware_msgs::MotorConfigServiceRequest :md5sum-) "0aa4adc3b3f52f02aacbfa621182887c")
(setf (get roboy_middleware_msgs::MotorConfigServiceRequest :datatype-) "roboy_middleware_msgs/MotorConfigServiceRequest")
(setf (get roboy_middleware_msgs::MotorConfigServiceRequest :definition-)
      "MotorConfig config

================================================================================
MSG: roboy_middleware_msgs/MotorConfig
int32 id
uint8[] motors
uint8[] control_mode
int32[] output_pos_max
int32[] output_neg_max
int32[] sp_pos_max
int32[] sp_neg_max
uint16[] kp
uint16[] ki
uint16[] kd
uint16[] forward_gain
uint16[] dead_band
int32[] integral_pos_max
int32[] integral_neg_max
int32[] output_divider
---
int32[] mode

")

(setf (get roboy_middleware_msgs::MotorConfigServiceResponse :md5sum-) "0aa4adc3b3f52f02aacbfa621182887c")
(setf (get roboy_middleware_msgs::MotorConfigServiceResponse :datatype-) "roboy_middleware_msgs/MotorConfigServiceResponse")
(setf (get roboy_middleware_msgs::MotorConfigServiceResponse :definition-)
      "MotorConfig config

================================================================================
MSG: roboy_middleware_msgs/MotorConfig
int32 id
uint8[] motors
uint8[] control_mode
int32[] output_pos_max
int32[] output_neg_max
int32[] sp_pos_max
int32[] sp_neg_max
uint16[] kp
uint16[] ki
uint16[] kd
uint16[] forward_gain
uint16[] dead_band
int32[] integral_pos_max
int32[] integral_neg_max
int32[] output_divider
---
int32[] mode

")



(provide :roboy_middleware_msgs/MotorConfigService "0aa4adc3b3f52f02aacbfa621182887c")


