; Auto-generated. Do not edit!


(cl:in-package roboy_middleware_msgs-srv)


;//! \htmlinclude ControlMode-request.msg.html

(cl:defclass <ControlMode-request> (roslisp-msg-protocol:ros-message)
  ((control_mode
    :reader control_mode
    :initarg :control_mode
    :type cl:integer
    :initform 0)
   (set_point
    :reader set_point
    :initarg :set_point
    :type cl:integer
    :initform 0)
   (motor_id
    :reader motor_id
    :initarg :motor_id
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass ControlMode-request (<ControlMode-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ControlMode-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ControlMode-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name roboy_middleware_msgs-srv:<ControlMode-request> is deprecated: use roboy_middleware_msgs-srv:ControlMode-request instead.")))

(cl:ensure-generic-function 'control_mode-val :lambda-list '(m))
(cl:defmethod control_mode-val ((m <ControlMode-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader roboy_middleware_msgs-srv:control_mode-val is deprecated.  Use roboy_middleware_msgs-srv:control_mode instead.")
  (control_mode m))

(cl:ensure-generic-function 'set_point-val :lambda-list '(m))
(cl:defmethod set_point-val ((m <ControlMode-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader roboy_middleware_msgs-srv:set_point-val is deprecated.  Use roboy_middleware_msgs-srv:set_point instead.")
  (set_point m))

(cl:ensure-generic-function 'motor_id-val :lambda-list '(m))
(cl:defmethod motor_id-val ((m <ControlMode-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader roboy_middleware_msgs-srv:motor_id-val is deprecated.  Use roboy_middleware_msgs-srv:motor_id instead.")
  (motor_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ControlMode-request>) ostream)
  "Serializes a message object of type '<ControlMode-request>"
  (cl:let* ((signed (cl:slot-value msg 'control_mode)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'set_point)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'motor_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream))
   (cl:slot-value msg 'motor_id))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ControlMode-request>) istream)
  "Deserializes a message object of type '<ControlMode-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'control_mode) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'set_point) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'motor_id) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'motor_id)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ControlMode-request>)))
  "Returns string type for a service object of type '<ControlMode-request>"
  "roboy_middleware_msgs/ControlModeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControlMode-request)))
  "Returns string type for a service object of type 'ControlMode-request"
  "roboy_middleware_msgs/ControlModeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ControlMode-request>)))
  "Returns md5sum for a message object of type '<ControlMode-request>"
  "3bf3f3c500bf7df65355716be531bee1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ControlMode-request)))
  "Returns md5sum for a message object of type 'ControlMode-request"
  "3bf3f3c500bf7df65355716be531bee1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ControlMode-request>)))
  "Returns full string definition for message of type '<ControlMode-request>"
  (cl:format cl:nil "int32 control_mode~%int32 set_point~%~%uint16[] motor_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ControlMode-request)))
  "Returns full string definition for message of type 'ControlMode-request"
  (cl:format cl:nil "int32 control_mode~%int32 set_point~%~%uint16[] motor_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ControlMode-request>))
  (cl:+ 0
     4
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'motor_id) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ControlMode-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ControlMode-request
    (cl:cons ':control_mode (control_mode msg))
    (cl:cons ':set_point (set_point msg))
    (cl:cons ':motor_id (motor_id msg))
))
;//! \htmlinclude ControlMode-response.msg.html

(cl:defclass <ControlMode-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass ControlMode-response (<ControlMode-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ControlMode-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ControlMode-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name roboy_middleware_msgs-srv:<ControlMode-response> is deprecated: use roboy_middleware_msgs-srv:ControlMode-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ControlMode-response>) ostream)
  "Serializes a message object of type '<ControlMode-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ControlMode-response>) istream)
  "Deserializes a message object of type '<ControlMode-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ControlMode-response>)))
  "Returns string type for a service object of type '<ControlMode-response>"
  "roboy_middleware_msgs/ControlModeResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControlMode-response)))
  "Returns string type for a service object of type 'ControlMode-response"
  "roboy_middleware_msgs/ControlModeResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ControlMode-response>)))
  "Returns md5sum for a message object of type '<ControlMode-response>"
  "3bf3f3c500bf7df65355716be531bee1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ControlMode-response)))
  "Returns md5sum for a message object of type 'ControlMode-response"
  "3bf3f3c500bf7df65355716be531bee1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ControlMode-response>)))
  "Returns full string definition for message of type '<ControlMode-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ControlMode-response)))
  "Returns full string definition for message of type 'ControlMode-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ControlMode-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ControlMode-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ControlMode-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ControlMode)))
  'ControlMode-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ControlMode)))
  'ControlMode-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControlMode)))
  "Returns string type for a service object of type '<ControlMode>"
  "roboy_middleware_msgs/ControlMode")