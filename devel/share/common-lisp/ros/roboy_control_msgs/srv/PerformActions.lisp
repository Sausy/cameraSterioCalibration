; Auto-generated. Do not edit!


(cl:in-package roboy_control_msgs-srv)


;//! \htmlinclude PerformActions-request.msg.html

(cl:defclass <PerformActions-request> (roslisp-msg-protocol:ros-message)
  ((actions
    :reader actions
    :initarg :actions
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element "")))
)

(cl:defclass PerformActions-request (<PerformActions-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PerformActions-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PerformActions-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name roboy_control_msgs-srv:<PerformActions-request> is deprecated: use roboy_control_msgs-srv:PerformActions-request instead.")))

(cl:ensure-generic-function 'actions-val :lambda-list '(m))
(cl:defmethod actions-val ((m <PerformActions-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader roboy_control_msgs-srv:actions-val is deprecated.  Use roboy_control_msgs-srv:actions instead.")
  (actions m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PerformActions-request>) ostream)
  "Serializes a message object of type '<PerformActions-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'actions))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'actions))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PerformActions-request>) istream)
  "Deserializes a message object of type '<PerformActions-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'actions) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'actions)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PerformActions-request>)))
  "Returns string type for a service object of type '<PerformActions-request>"
  "roboy_control_msgs/PerformActionsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PerformActions-request)))
  "Returns string type for a service object of type 'PerformActions-request"
  "roboy_control_msgs/PerformActionsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PerformActions-request>)))
  "Returns md5sum for a message object of type '<PerformActions-request>"
  "4c7ae25d13777401a5c311c271b76cf4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PerformActions-request)))
  "Returns md5sum for a message object of type 'PerformActions-request"
  "4c7ae25d13777401a5c311c271b76cf4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PerformActions-request>)))
  "Returns full string definition for message of type '<PerformActions-request>"
  (cl:format cl:nil "string[] actions # can be a trajectory name or pause (2_pause will pause for 2 seconds)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PerformActions-request)))
  "Returns full string definition for message of type 'PerformActions-request"
  (cl:format cl:nil "string[] actions # can be a trajectory name or pause (2_pause will pause for 2 seconds)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PerformActions-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'actions) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PerformActions-request>))
  "Converts a ROS message object to a list"
  (cl:list 'PerformActions-request
    (cl:cons ':actions (actions msg))
))
;//! \htmlinclude PerformActions-response.msg.html

(cl:defclass <PerformActions-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass PerformActions-response (<PerformActions-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PerformActions-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PerformActions-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name roboy_control_msgs-srv:<PerformActions-response> is deprecated: use roboy_control_msgs-srv:PerformActions-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <PerformActions-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader roboy_control_msgs-srv:success-val is deprecated.  Use roboy_control_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PerformActions-response>) ostream)
  "Serializes a message object of type '<PerformActions-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PerformActions-response>) istream)
  "Deserializes a message object of type '<PerformActions-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PerformActions-response>)))
  "Returns string type for a service object of type '<PerformActions-response>"
  "roboy_control_msgs/PerformActionsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PerformActions-response)))
  "Returns string type for a service object of type 'PerformActions-response"
  "roboy_control_msgs/PerformActionsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PerformActions-response>)))
  "Returns md5sum for a message object of type '<PerformActions-response>"
  "4c7ae25d13777401a5c311c271b76cf4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PerformActions-response)))
  "Returns md5sum for a message object of type 'PerformActions-response"
  "4c7ae25d13777401a5c311c271b76cf4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PerformActions-response>)))
  "Returns full string definition for message of type '<PerformActions-response>"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PerformActions-response)))
  "Returns full string definition for message of type 'PerformActions-response"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PerformActions-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PerformActions-response>))
  "Converts a ROS message object to a list"
  (cl:list 'PerformActions-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'PerformActions)))
  'PerformActions-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'PerformActions)))
  'PerformActions-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PerformActions)))
  "Returns string type for a service object of type '<PerformActions>"
  "roboy_control_msgs/PerformActions")