; Auto-generated. Do not edit!


(cl:in-package roboy_middleware_msgs-msg)


;//! \htmlinclude DarkRoomSensorV2.msg.html

(cl:defclass <DarkRoomSensorV2> (roslisp-msg-protocol:ros-message)
  ((object_id
    :reader object_id
    :initarg :object_id
    :type cl:string
    :initform "")
   (base
    :reader base
    :initarg :base
    :type cl:fixnum
    :initform 0)
   (SensorID
    :reader SensorID
    :initarg :SensorID
    :type cl:fixnum
    :initform 0)
   (elevation
    :reader elevation
    :initarg :elevation
    :type cl:float
    :initform 0.0)
   (azimuth
    :reader azimuth
    :initarg :azimuth
    :type cl:float
    :initform 0.0))
)

(cl:defclass DarkRoomSensorV2 (<DarkRoomSensorV2>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DarkRoomSensorV2>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DarkRoomSensorV2)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name roboy_middleware_msgs-msg:<DarkRoomSensorV2> is deprecated: use roboy_middleware_msgs-msg:DarkRoomSensorV2 instead.")))

(cl:ensure-generic-function 'object_id-val :lambda-list '(m))
(cl:defmethod object_id-val ((m <DarkRoomSensorV2>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader roboy_middleware_msgs-msg:object_id-val is deprecated.  Use roboy_middleware_msgs-msg:object_id instead.")
  (object_id m))

(cl:ensure-generic-function 'base-val :lambda-list '(m))
(cl:defmethod base-val ((m <DarkRoomSensorV2>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader roboy_middleware_msgs-msg:base-val is deprecated.  Use roboy_middleware_msgs-msg:base instead.")
  (base m))

(cl:ensure-generic-function 'SensorID-val :lambda-list '(m))
(cl:defmethod SensorID-val ((m <DarkRoomSensorV2>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader roboy_middleware_msgs-msg:SensorID-val is deprecated.  Use roboy_middleware_msgs-msg:SensorID instead.")
  (SensorID m))

(cl:ensure-generic-function 'elevation-val :lambda-list '(m))
(cl:defmethod elevation-val ((m <DarkRoomSensorV2>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader roboy_middleware_msgs-msg:elevation-val is deprecated.  Use roboy_middleware_msgs-msg:elevation instead.")
  (elevation m))

(cl:ensure-generic-function 'azimuth-val :lambda-list '(m))
(cl:defmethod azimuth-val ((m <DarkRoomSensorV2>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader roboy_middleware_msgs-msg:azimuth-val is deprecated.  Use roboy_middleware_msgs-msg:azimuth instead.")
  (azimuth m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DarkRoomSensorV2>) ostream)
  "Serializes a message object of type '<DarkRoomSensorV2>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'object_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'object_id))
  (cl:let* ((signed (cl:slot-value msg 'base)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'SensorID)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'elevation))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'azimuth))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DarkRoomSensorV2>) istream)
  "Deserializes a message object of type '<DarkRoomSensorV2>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'object_id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'object_id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'base) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'SensorID) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'elevation) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'azimuth) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DarkRoomSensorV2>)))
  "Returns string type for a message object of type '<DarkRoomSensorV2>"
  "roboy_middleware_msgs/DarkRoomSensorV2")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DarkRoomSensorV2)))
  "Returns string type for a message object of type 'DarkRoomSensorV2"
  "roboy_middleware_msgs/DarkRoomSensorV2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DarkRoomSensorV2>)))
  "Returns md5sum for a message object of type '<DarkRoomSensorV2>"
  "33742585e0d9d4936d60498b07ad60de")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DarkRoomSensorV2)))
  "Returns md5sum for a message object of type 'DarkRoomSensorV2"
  "33742585e0d9d4936d60498b07ad60de")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DarkRoomSensorV2>)))
  "Returns full string definition for message of type '<DarkRoomSensorV2>"
  (cl:format cl:nil "string object_id~%int8 base~%int8 SensorID~%float64 elevation~%float64 azimuth~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DarkRoomSensorV2)))
  "Returns full string definition for message of type 'DarkRoomSensorV2"
  (cl:format cl:nil "string object_id~%int8 base~%int8 SensorID~%float64 elevation~%float64 azimuth~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DarkRoomSensorV2>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'object_id))
     1
     1
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DarkRoomSensorV2>))
  "Converts a ROS message object to a list"
  (cl:list 'DarkRoomSensorV2
    (cl:cons ':object_id (object_id msg))
    (cl:cons ':base (base msg))
    (cl:cons ':SensorID (SensorID msg))
    (cl:cons ':elevation (elevation msg))
    (cl:cons ':azimuth (azimuth msg))
))
