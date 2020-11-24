; Auto-generated. Do not edit!


(cl:in-package roboy_middleware_msgs-srv)


;//! \htmlinclude MotorConfigService-request.msg.html

(cl:defclass <MotorConfigService-request> (roslisp-msg-protocol:ros-message)
  ((config
    :reader config
    :initarg :config
    :type roboy_middleware_msgs-msg:MotorConfig
    :initform (cl:make-instance 'roboy_middleware_msgs-msg:MotorConfig)))
)

(cl:defclass MotorConfigService-request (<MotorConfigService-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MotorConfigService-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MotorConfigService-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name roboy_middleware_msgs-srv:<MotorConfigService-request> is deprecated: use roboy_middleware_msgs-srv:MotorConfigService-request instead.")))

(cl:ensure-generic-function 'config-val :lambda-list '(m))
(cl:defmethod config-val ((m <MotorConfigService-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader roboy_middleware_msgs-srv:config-val is deprecated.  Use roboy_middleware_msgs-srv:config instead.")
  (config m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MotorConfigService-request>) ostream)
  "Serializes a message object of type '<MotorConfigService-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'config) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MotorConfigService-request>) istream)
  "Deserializes a message object of type '<MotorConfigService-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'config) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MotorConfigService-request>)))
  "Returns string type for a service object of type '<MotorConfigService-request>"
  "roboy_middleware_msgs/MotorConfigServiceRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MotorConfigService-request)))
  "Returns string type for a service object of type 'MotorConfigService-request"
  "roboy_middleware_msgs/MotorConfigServiceRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MotorConfigService-request>)))
  "Returns md5sum for a message object of type '<MotorConfigService-request>"
  "0aa4adc3b3f52f02aacbfa621182887c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MotorConfigService-request)))
  "Returns md5sum for a message object of type 'MotorConfigService-request"
  "0aa4adc3b3f52f02aacbfa621182887c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MotorConfigService-request>)))
  "Returns full string definition for message of type '<MotorConfigService-request>"
  (cl:format cl:nil "MotorConfig config~%~%================================================================================~%MSG: roboy_middleware_msgs/MotorConfig~%int32 id~%uint8[] motors~%uint8[] control_mode~%int32[] output_pos_max~%int32[] output_neg_max~%int32[] sp_pos_max~%int32[] sp_neg_max~%uint16[] kp~%uint16[] ki~%uint16[] kd~%uint16[] forward_gain~%uint16[] dead_band~%int32[] integral_pos_max~%int32[] integral_neg_max~%int32[] output_divider~%int32[] setpoint~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MotorConfigService-request)))
  "Returns full string definition for message of type 'MotorConfigService-request"
  (cl:format cl:nil "MotorConfig config~%~%================================================================================~%MSG: roboy_middleware_msgs/MotorConfig~%int32 id~%uint8[] motors~%uint8[] control_mode~%int32[] output_pos_max~%int32[] output_neg_max~%int32[] sp_pos_max~%int32[] sp_neg_max~%uint16[] kp~%uint16[] ki~%uint16[] kd~%uint16[] forward_gain~%uint16[] dead_band~%int32[] integral_pos_max~%int32[] integral_neg_max~%int32[] output_divider~%int32[] setpoint~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MotorConfigService-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'config))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MotorConfigService-request>))
  "Converts a ROS message object to a list"
  (cl:list 'MotorConfigService-request
    (cl:cons ':config (config msg))
))
;//! \htmlinclude MotorConfigService-response.msg.html

(cl:defclass <MotorConfigService-response> (roslisp-msg-protocol:ros-message)
  ((mode
    :reader mode
    :initarg :mode
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0)))
)

(cl:defclass MotorConfigService-response (<MotorConfigService-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MotorConfigService-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MotorConfigService-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name roboy_middleware_msgs-srv:<MotorConfigService-response> is deprecated: use roboy_middleware_msgs-srv:MotorConfigService-response instead.")))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <MotorConfigService-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader roboy_middleware_msgs-srv:mode-val is deprecated.  Use roboy_middleware_msgs-srv:mode instead.")
  (mode m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MotorConfigService-response>) ostream)
  "Serializes a message object of type '<MotorConfigService-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'mode))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'mode))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MotorConfigService-response>) istream)
  "Deserializes a message object of type '<MotorConfigService-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'mode) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'mode)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296)))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MotorConfigService-response>)))
  "Returns string type for a service object of type '<MotorConfigService-response>"
  "roboy_middleware_msgs/MotorConfigServiceResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MotorConfigService-response)))
  "Returns string type for a service object of type 'MotorConfigService-response"
  "roboy_middleware_msgs/MotorConfigServiceResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MotorConfigService-response>)))
  "Returns md5sum for a message object of type '<MotorConfigService-response>"
  "0aa4adc3b3f52f02aacbfa621182887c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MotorConfigService-response)))
  "Returns md5sum for a message object of type 'MotorConfigService-response"
  "0aa4adc3b3f52f02aacbfa621182887c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MotorConfigService-response>)))
  "Returns full string definition for message of type '<MotorConfigService-response>"
  (cl:format cl:nil "int32[] mode~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MotorConfigService-response)))
  "Returns full string definition for message of type 'MotorConfigService-response"
  (cl:format cl:nil "int32[] mode~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MotorConfigService-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'mode) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MotorConfigService-response>))
  "Converts a ROS message object to a list"
  (cl:list 'MotorConfigService-response
    (cl:cons ':mode (mode msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'MotorConfigService)))
  'MotorConfigService-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'MotorConfigService)))
  'MotorConfigService-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MotorConfigService)))
  "Returns string type for a service object of type '<MotorConfigService>"
  "roboy_middleware_msgs/MotorConfigService")