; Auto-generated. Do not edit!


(cl:in-package roboy_middleware_msgs-srv)


;//! \htmlinclude XL320-request.msg.html

(cl:defclass <XL320-request> (roslisp-msg-protocol:ros-message)
  ((type
    :reader type
    :initarg :type
    :type cl:boolean
    :initform cl:nil)
   (motor
    :reader motor
    :initarg :motor
    :type cl:fixnum
    :initform 0)
   (address
    :reader address
    :initarg :address
    :type cl:fixnum
    :initform 0)
   (value
    :reader value
    :initarg :value
    :type cl:fixnum
    :initform 0))
)

(cl:defclass XL320-request (<XL320-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <XL320-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'XL320-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name roboy_middleware_msgs-srv:<XL320-request> is deprecated: use roboy_middleware_msgs-srv:XL320-request instead.")))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <XL320-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader roboy_middleware_msgs-srv:type-val is deprecated.  Use roboy_middleware_msgs-srv:type instead.")
  (type m))

(cl:ensure-generic-function 'motor-val :lambda-list '(m))
(cl:defmethod motor-val ((m <XL320-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader roboy_middleware_msgs-srv:motor-val is deprecated.  Use roboy_middleware_msgs-srv:motor instead.")
  (motor m))

(cl:ensure-generic-function 'address-val :lambda-list '(m))
(cl:defmethod address-val ((m <XL320-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader roboy_middleware_msgs-srv:address-val is deprecated.  Use roboy_middleware_msgs-srv:address instead.")
  (address m))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <XL320-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader roboy_middleware_msgs-srv:value-val is deprecated.  Use roboy_middleware_msgs-srv:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <XL320-request>) ostream)
  "Serializes a message object of type '<XL320-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'type) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'motor)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'address)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'value)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <XL320-request>) istream)
  "Deserializes a message object of type '<XL320-request>"
    (cl:setf (cl:slot-value msg 'type) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'motor)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'address)) (cl:read-byte istream))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'value) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<XL320-request>)))
  "Returns string type for a service object of type '<XL320-request>"
  "roboy_middleware_msgs/XL320Request")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'XL320-request)))
  "Returns string type for a service object of type 'XL320-request"
  "roboy_middleware_msgs/XL320Request")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<XL320-request>)))
  "Returns md5sum for a message object of type '<XL320-request>"
  "2df206e3bbdc218ed0e86d48777a9fbf")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'XL320-request)))
  "Returns md5sum for a message object of type 'XL320-request"
  "2df206e3bbdc218ed0e86d48777a9fbf")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<XL320-request>)))
  "Returns full string definition for message of type '<XL320-request>"
  (cl:format cl:nil "~%bool type~%uint8 motor~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%uint8 address~%int16 value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'XL320-request)))
  "Returns full string definition for message of type 'XL320-request"
  (cl:format cl:nil "~%bool type~%uint8 motor~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%uint8 address~%int16 value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <XL320-request>))
  (cl:+ 0
     1
     1
     1
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <XL320-request>))
  "Converts a ROS message object to a list"
  (cl:list 'XL320-request
    (cl:cons ':type (type msg))
    (cl:cons ':motor (motor msg))
    (cl:cons ':address (address msg))
    (cl:cons ':value (value msg))
))
;//! \htmlinclude XL320-response.msg.html

(cl:defclass <XL320-response> (roslisp-msg-protocol:ros-message)
  ((value
    :reader value
    :initarg :value
    :type cl:fixnum
    :initform 0))
)

(cl:defclass XL320-response (<XL320-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <XL320-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'XL320-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name roboy_middleware_msgs-srv:<XL320-response> is deprecated: use roboy_middleware_msgs-srv:XL320-response instead.")))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <XL320-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader roboy_middleware_msgs-srv:value-val is deprecated.  Use roboy_middleware_msgs-srv:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <XL320-response>) ostream)
  "Serializes a message object of type '<XL320-response>"
  (cl:let* ((signed (cl:slot-value msg 'value)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <XL320-response>) istream)
  "Deserializes a message object of type '<XL320-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'value) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<XL320-response>)))
  "Returns string type for a service object of type '<XL320-response>"
  "roboy_middleware_msgs/XL320Response")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'XL320-response)))
  "Returns string type for a service object of type 'XL320-response"
  "roboy_middleware_msgs/XL320Response")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<XL320-response>)))
  "Returns md5sum for a message object of type '<XL320-response>"
  "2df206e3bbdc218ed0e86d48777a9fbf")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'XL320-response)))
  "Returns md5sum for a message object of type 'XL320-response"
  "2df206e3bbdc218ed0e86d48777a9fbf")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<XL320-response>)))
  "Returns full string definition for message of type '<XL320-response>"
  (cl:format cl:nil "int16 value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'XL320-response)))
  "Returns full string definition for message of type 'XL320-response"
  (cl:format cl:nil "int16 value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <XL320-response>))
  (cl:+ 0
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <XL320-response>))
  "Converts a ROS message object to a list"
  (cl:list 'XL320-response
    (cl:cons ':value (value msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'XL320)))
  'XL320-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'XL320)))
  'XL320-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'XL320)))
  "Returns string type for a service object of type '<XL320>"
  "roboy_middleware_msgs/XL320")