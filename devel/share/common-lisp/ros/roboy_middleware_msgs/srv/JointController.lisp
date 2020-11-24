; Auto-generated. Do not edit!


(cl:in-package roboy_middleware_msgs-srv)


;//! \htmlinclude JointController-request.msg.html

(cl:defclass <JointController-request> (roslisp-msg-protocol:ros-message)
  ((elbow_control_enable
    :reader elbow_control_enable
    :initarg :elbow_control_enable
    :type cl:boolean
    :initform cl:nil)
   (kp_elbow_joint
    :reader kp_elbow_joint
    :initarg :kp_elbow_joint
    :type cl:integer
    :initform 0)
   (kd_elbow_joint
    :reader kd_elbow_joint
    :initarg :kd_elbow_joint
    :type cl:integer
    :initform 0)
   (kp_elbow_agonist
    :reader kp_elbow_agonist
    :initarg :kp_elbow_agonist
    :type cl:integer
    :initform 0)
   (kd_elbow_agonist
    :reader kd_elbow_agonist
    :initarg :kd_elbow_agonist
    :type cl:integer
    :initform 0)
   (kp_elbow_antagonist
    :reader kp_elbow_antagonist
    :initarg :kp_elbow_antagonist
    :type cl:integer
    :initform 0)
   (kd_elbow_antagonist
    :reader kd_elbow_antagonist
    :initarg :kd_elbow_antagonist
    :type cl:integer
    :initform 0)
   (elbow_deadband
    :reader elbow_deadband
    :initarg :elbow_deadband
    :type cl:integer
    :initform 0)
   (elbow_pretension
    :reader elbow_pretension
    :initarg :elbow_pretension
    :type cl:integer
    :initform 0)
   (wrist_control_enable
    :reader wrist_control_enable
    :initarg :wrist_control_enable
    :type cl:boolean
    :initform cl:nil)
   (kp_wrist_joint
    :reader kp_wrist_joint
    :initarg :kp_wrist_joint
    :type cl:integer
    :initform 0)
   (kd_wrist_joint
    :reader kd_wrist_joint
    :initarg :kd_wrist_joint
    :type cl:integer
    :initform 0)
   (kp_wrist_agonist
    :reader kp_wrist_agonist
    :initarg :kp_wrist_agonist
    :type cl:integer
    :initform 0)
   (kd_wrist_agonist
    :reader kd_wrist_agonist
    :initarg :kd_wrist_agonist
    :type cl:integer
    :initform 0)
   (kp_wrist_antagonist
    :reader kp_wrist_antagonist
    :initarg :kp_wrist_antagonist
    :type cl:integer
    :initform 0)
   (kd_wrist_antagonist
    :reader kd_wrist_antagonist
    :initarg :kd_wrist_antagonist
    :type cl:integer
    :initform 0)
   (wrist_deadband
    :reader wrist_deadband
    :initarg :wrist_deadband
    :type cl:integer
    :initform 0)
   (wrist_pretension
    :reader wrist_pretension
    :initarg :wrist_pretension
    :type cl:integer
    :initform 0))
)

(cl:defclass JointController-request (<JointController-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <JointController-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'JointController-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name roboy_middleware_msgs-srv:<JointController-request> is deprecated: use roboy_middleware_msgs-srv:JointController-request instead.")))

(cl:ensure-generic-function 'elbow_control_enable-val :lambda-list '(m))
(cl:defmethod elbow_control_enable-val ((m <JointController-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader roboy_middleware_msgs-srv:elbow_control_enable-val is deprecated.  Use roboy_middleware_msgs-srv:elbow_control_enable instead.")
  (elbow_control_enable m))

(cl:ensure-generic-function 'kp_elbow_joint-val :lambda-list '(m))
(cl:defmethod kp_elbow_joint-val ((m <JointController-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader roboy_middleware_msgs-srv:kp_elbow_joint-val is deprecated.  Use roboy_middleware_msgs-srv:kp_elbow_joint instead.")
  (kp_elbow_joint m))

(cl:ensure-generic-function 'kd_elbow_joint-val :lambda-list '(m))
(cl:defmethod kd_elbow_joint-val ((m <JointController-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader roboy_middleware_msgs-srv:kd_elbow_joint-val is deprecated.  Use roboy_middleware_msgs-srv:kd_elbow_joint instead.")
  (kd_elbow_joint m))

(cl:ensure-generic-function 'kp_elbow_agonist-val :lambda-list '(m))
(cl:defmethod kp_elbow_agonist-val ((m <JointController-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader roboy_middleware_msgs-srv:kp_elbow_agonist-val is deprecated.  Use roboy_middleware_msgs-srv:kp_elbow_agonist instead.")
  (kp_elbow_agonist m))

(cl:ensure-generic-function 'kd_elbow_agonist-val :lambda-list '(m))
(cl:defmethod kd_elbow_agonist-val ((m <JointController-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader roboy_middleware_msgs-srv:kd_elbow_agonist-val is deprecated.  Use roboy_middleware_msgs-srv:kd_elbow_agonist instead.")
  (kd_elbow_agonist m))

(cl:ensure-generic-function 'kp_elbow_antagonist-val :lambda-list '(m))
(cl:defmethod kp_elbow_antagonist-val ((m <JointController-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader roboy_middleware_msgs-srv:kp_elbow_antagonist-val is deprecated.  Use roboy_middleware_msgs-srv:kp_elbow_antagonist instead.")
  (kp_elbow_antagonist m))

(cl:ensure-generic-function 'kd_elbow_antagonist-val :lambda-list '(m))
(cl:defmethod kd_elbow_antagonist-val ((m <JointController-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader roboy_middleware_msgs-srv:kd_elbow_antagonist-val is deprecated.  Use roboy_middleware_msgs-srv:kd_elbow_antagonist instead.")
  (kd_elbow_antagonist m))

(cl:ensure-generic-function 'elbow_deadband-val :lambda-list '(m))
(cl:defmethod elbow_deadband-val ((m <JointController-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader roboy_middleware_msgs-srv:elbow_deadband-val is deprecated.  Use roboy_middleware_msgs-srv:elbow_deadband instead.")
  (elbow_deadband m))

(cl:ensure-generic-function 'elbow_pretension-val :lambda-list '(m))
(cl:defmethod elbow_pretension-val ((m <JointController-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader roboy_middleware_msgs-srv:elbow_pretension-val is deprecated.  Use roboy_middleware_msgs-srv:elbow_pretension instead.")
  (elbow_pretension m))

(cl:ensure-generic-function 'wrist_control_enable-val :lambda-list '(m))
(cl:defmethod wrist_control_enable-val ((m <JointController-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader roboy_middleware_msgs-srv:wrist_control_enable-val is deprecated.  Use roboy_middleware_msgs-srv:wrist_control_enable instead.")
  (wrist_control_enable m))

(cl:ensure-generic-function 'kp_wrist_joint-val :lambda-list '(m))
(cl:defmethod kp_wrist_joint-val ((m <JointController-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader roboy_middleware_msgs-srv:kp_wrist_joint-val is deprecated.  Use roboy_middleware_msgs-srv:kp_wrist_joint instead.")
  (kp_wrist_joint m))

(cl:ensure-generic-function 'kd_wrist_joint-val :lambda-list '(m))
(cl:defmethod kd_wrist_joint-val ((m <JointController-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader roboy_middleware_msgs-srv:kd_wrist_joint-val is deprecated.  Use roboy_middleware_msgs-srv:kd_wrist_joint instead.")
  (kd_wrist_joint m))

(cl:ensure-generic-function 'kp_wrist_agonist-val :lambda-list '(m))
(cl:defmethod kp_wrist_agonist-val ((m <JointController-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader roboy_middleware_msgs-srv:kp_wrist_agonist-val is deprecated.  Use roboy_middleware_msgs-srv:kp_wrist_agonist instead.")
  (kp_wrist_agonist m))

(cl:ensure-generic-function 'kd_wrist_agonist-val :lambda-list '(m))
(cl:defmethod kd_wrist_agonist-val ((m <JointController-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader roboy_middleware_msgs-srv:kd_wrist_agonist-val is deprecated.  Use roboy_middleware_msgs-srv:kd_wrist_agonist instead.")
  (kd_wrist_agonist m))

(cl:ensure-generic-function 'kp_wrist_antagonist-val :lambda-list '(m))
(cl:defmethod kp_wrist_antagonist-val ((m <JointController-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader roboy_middleware_msgs-srv:kp_wrist_antagonist-val is deprecated.  Use roboy_middleware_msgs-srv:kp_wrist_antagonist instead.")
  (kp_wrist_antagonist m))

(cl:ensure-generic-function 'kd_wrist_antagonist-val :lambda-list '(m))
(cl:defmethod kd_wrist_antagonist-val ((m <JointController-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader roboy_middleware_msgs-srv:kd_wrist_antagonist-val is deprecated.  Use roboy_middleware_msgs-srv:kd_wrist_antagonist instead.")
  (kd_wrist_antagonist m))

(cl:ensure-generic-function 'wrist_deadband-val :lambda-list '(m))
(cl:defmethod wrist_deadband-val ((m <JointController-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader roboy_middleware_msgs-srv:wrist_deadband-val is deprecated.  Use roboy_middleware_msgs-srv:wrist_deadband instead.")
  (wrist_deadband m))

(cl:ensure-generic-function 'wrist_pretension-val :lambda-list '(m))
(cl:defmethod wrist_pretension-val ((m <JointController-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader roboy_middleware_msgs-srv:wrist_pretension-val is deprecated.  Use roboy_middleware_msgs-srv:wrist_pretension instead.")
  (wrist_pretension m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <JointController-request>) ostream)
  "Serializes a message object of type '<JointController-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'elbow_control_enable) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'kp_elbow_joint)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'kd_elbow_joint)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'kp_elbow_agonist)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'kd_elbow_agonist)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'kp_elbow_antagonist)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'kd_elbow_antagonist)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'elbow_deadband)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'elbow_pretension)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'wrist_control_enable) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'kp_wrist_joint)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'kd_wrist_joint)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'kp_wrist_agonist)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'kd_wrist_agonist)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'kp_wrist_antagonist)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'kd_wrist_antagonist)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'wrist_deadband)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'wrist_pretension)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <JointController-request>) istream)
  "Deserializes a message object of type '<JointController-request>"
    (cl:setf (cl:slot-value msg 'elbow_control_enable) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'kp_elbow_joint) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'kd_elbow_joint) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'kp_elbow_agonist) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'kd_elbow_agonist) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'kp_elbow_antagonist) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'kd_elbow_antagonist) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'elbow_deadband) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'elbow_pretension) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:setf (cl:slot-value msg 'wrist_control_enable) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'kp_wrist_joint) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'kd_wrist_joint) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'kp_wrist_agonist) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'kd_wrist_agonist) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'kp_wrist_antagonist) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'kd_wrist_antagonist) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'wrist_deadband) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'wrist_pretension) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<JointController-request>)))
  "Returns string type for a service object of type '<JointController-request>"
  "roboy_middleware_msgs/JointControllerRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'JointController-request)))
  "Returns string type for a service object of type 'JointController-request"
  "roboy_middleware_msgs/JointControllerRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<JointController-request>)))
  "Returns md5sum for a message object of type '<JointController-request>"
  "8c2b216fbd74af3eb65d1ca2f72cefc3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'JointController-request)))
  "Returns md5sum for a message object of type 'JointController-request"
  "8c2b216fbd74af3eb65d1ca2f72cefc3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<JointController-request>)))
  "Returns full string definition for message of type '<JointController-request>"
  (cl:format cl:nil "bool elbow_control_enable~%int32 kp_elbow_joint~%int32 kd_elbow_joint~%int32 kp_elbow_agonist~%int32 kd_elbow_agonist~%int32 kp_elbow_antagonist~%int32 kd_elbow_antagonist~%int32 elbow_deadband~%int32 elbow_pretension~%bool wrist_control_enable~%int32 kp_wrist_joint~%int32 kd_wrist_joint~%int32 kp_wrist_agonist~%int32 kd_wrist_agonist~%int32 kp_wrist_antagonist~%int32 kd_wrist_antagonist~%int32 wrist_deadband~%int32 wrist_pretension~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'JointController-request)))
  "Returns full string definition for message of type 'JointController-request"
  (cl:format cl:nil "bool elbow_control_enable~%int32 kp_elbow_joint~%int32 kd_elbow_joint~%int32 kp_elbow_agonist~%int32 kd_elbow_agonist~%int32 kp_elbow_antagonist~%int32 kd_elbow_antagonist~%int32 elbow_deadband~%int32 elbow_pretension~%bool wrist_control_enable~%int32 kp_wrist_joint~%int32 kd_wrist_joint~%int32 kp_wrist_agonist~%int32 kd_wrist_agonist~%int32 kp_wrist_antagonist~%int32 kd_wrist_antagonist~%int32 wrist_deadband~%int32 wrist_pretension~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <JointController-request>))
  (cl:+ 0
     1
     4
     4
     4
     4
     4
     4
     4
     4
     1
     4
     4
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <JointController-request>))
  "Converts a ROS message object to a list"
  (cl:list 'JointController-request
    (cl:cons ':elbow_control_enable (elbow_control_enable msg))
    (cl:cons ':kp_elbow_joint (kp_elbow_joint msg))
    (cl:cons ':kd_elbow_joint (kd_elbow_joint msg))
    (cl:cons ':kp_elbow_agonist (kp_elbow_agonist msg))
    (cl:cons ':kd_elbow_agonist (kd_elbow_agonist msg))
    (cl:cons ':kp_elbow_antagonist (kp_elbow_antagonist msg))
    (cl:cons ':kd_elbow_antagonist (kd_elbow_antagonist msg))
    (cl:cons ':elbow_deadband (elbow_deadband msg))
    (cl:cons ':elbow_pretension (elbow_pretension msg))
    (cl:cons ':wrist_control_enable (wrist_control_enable msg))
    (cl:cons ':kp_wrist_joint (kp_wrist_joint msg))
    (cl:cons ':kd_wrist_joint (kd_wrist_joint msg))
    (cl:cons ':kp_wrist_agonist (kp_wrist_agonist msg))
    (cl:cons ':kd_wrist_agonist (kd_wrist_agonist msg))
    (cl:cons ':kp_wrist_antagonist (kp_wrist_antagonist msg))
    (cl:cons ':kd_wrist_antagonist (kd_wrist_antagonist msg))
    (cl:cons ':wrist_deadband (wrist_deadband msg))
    (cl:cons ':wrist_pretension (wrist_pretension msg))
))
;//! \htmlinclude JointController-response.msg.html

(cl:defclass <JointController-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass JointController-response (<JointController-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <JointController-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'JointController-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name roboy_middleware_msgs-srv:<JointController-response> is deprecated: use roboy_middleware_msgs-srv:JointController-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <JointController-response>) ostream)
  "Serializes a message object of type '<JointController-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <JointController-response>) istream)
  "Deserializes a message object of type '<JointController-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<JointController-response>)))
  "Returns string type for a service object of type '<JointController-response>"
  "roboy_middleware_msgs/JointControllerResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'JointController-response)))
  "Returns string type for a service object of type 'JointController-response"
  "roboy_middleware_msgs/JointControllerResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<JointController-response>)))
  "Returns md5sum for a message object of type '<JointController-response>"
  "8c2b216fbd74af3eb65d1ca2f72cefc3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'JointController-response)))
  "Returns md5sum for a message object of type 'JointController-response"
  "8c2b216fbd74af3eb65d1ca2f72cefc3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<JointController-response>)))
  "Returns full string definition for message of type '<JointController-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'JointController-response)))
  "Returns full string definition for message of type 'JointController-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <JointController-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <JointController-response>))
  "Converts a ROS message object to a list"
  (cl:list 'JointController-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'JointController)))
  'JointController-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'JointController)))
  'JointController-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'JointController)))
  "Returns string type for a service object of type '<JointController>"
  "roboy_middleware_msgs/JointController")