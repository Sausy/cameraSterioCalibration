;; Auto-generated. Do not edit!


(when (boundp 'roboy_middleware_msgs::MotorConfig)
  (if (not (find-package "ROBOY_MIDDLEWARE_MSGS"))
    (make-package "ROBOY_MIDDLEWARE_MSGS"))
  (shadow 'MotorConfig (find-package "ROBOY_MIDDLEWARE_MSGS")))
(unless (find-package "ROBOY_MIDDLEWARE_MSGS::MOTORCONFIG")
  (make-package "ROBOY_MIDDLEWARE_MSGS::MOTORCONFIG"))

(in-package "ROS")
;;//! \htmlinclude MotorConfig.msg.html


(defclass roboy_middleware_msgs::MotorConfig
  :super ros::object
  :slots (_id _motors _control_mode _output_pos_max _output_neg_max _sp_pos_max _sp_neg_max _kp _ki _kd _forward_gain _dead_band _integral_pos_max _integral_neg_max _output_divider _setpoint ))

(defmethod roboy_middleware_msgs::MotorConfig
  (:init
   (&key
    ((:id __id) 0)
    ((:motors __motors) (make-array 0 :initial-element 0 :element-type :char))
    ((:control_mode __control_mode) (make-array 0 :initial-element 0 :element-type :char))
    ((:output_pos_max __output_pos_max) (make-array 0 :initial-element 0 :element-type :integer))
    ((:output_neg_max __output_neg_max) (make-array 0 :initial-element 0 :element-type :integer))
    ((:sp_pos_max __sp_pos_max) (make-array 0 :initial-element 0 :element-type :integer))
    ((:sp_neg_max __sp_neg_max) (make-array 0 :initial-element 0 :element-type :integer))
    ((:kp __kp) (make-array 0 :initial-element 0 :element-type :integer))
    ((:ki __ki) (make-array 0 :initial-element 0 :element-type :integer))
    ((:kd __kd) (make-array 0 :initial-element 0 :element-type :integer))
    ((:forward_gain __forward_gain) (make-array 0 :initial-element 0 :element-type :integer))
    ((:dead_band __dead_band) (make-array 0 :initial-element 0 :element-type :integer))
    ((:integral_pos_max __integral_pos_max) (make-array 0 :initial-element 0 :element-type :integer))
    ((:integral_neg_max __integral_neg_max) (make-array 0 :initial-element 0 :element-type :integer))
    ((:output_divider __output_divider) (make-array 0 :initial-element 0 :element-type :integer))
    ((:setpoint __setpoint) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _id (round __id))
   (setq _motors __motors)
   (setq _control_mode __control_mode)
   (setq _output_pos_max __output_pos_max)
   (setq _output_neg_max __output_neg_max)
   (setq _sp_pos_max __sp_pos_max)
   (setq _sp_neg_max __sp_neg_max)
   (setq _kp __kp)
   (setq _ki __ki)
   (setq _kd __kd)
   (setq _forward_gain __forward_gain)
   (setq _dead_band __dead_band)
   (setq _integral_pos_max __integral_pos_max)
   (setq _integral_neg_max __integral_neg_max)
   (setq _output_divider __output_divider)
   (setq _setpoint __setpoint)
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:motors
   (&optional __motors)
   (if __motors (setq _motors __motors)) _motors)
  (:control_mode
   (&optional __control_mode)
   (if __control_mode (setq _control_mode __control_mode)) _control_mode)
  (:output_pos_max
   (&optional __output_pos_max)
   (if __output_pos_max (setq _output_pos_max __output_pos_max)) _output_pos_max)
  (:output_neg_max
   (&optional __output_neg_max)
   (if __output_neg_max (setq _output_neg_max __output_neg_max)) _output_neg_max)
  (:sp_pos_max
   (&optional __sp_pos_max)
   (if __sp_pos_max (setq _sp_pos_max __sp_pos_max)) _sp_pos_max)
  (:sp_neg_max
   (&optional __sp_neg_max)
   (if __sp_neg_max (setq _sp_neg_max __sp_neg_max)) _sp_neg_max)
  (:kp
   (&optional __kp)
   (if __kp (setq _kp __kp)) _kp)
  (:ki
   (&optional __ki)
   (if __ki (setq _ki __ki)) _ki)
  (:kd
   (&optional __kd)
   (if __kd (setq _kd __kd)) _kd)
  (:forward_gain
   (&optional __forward_gain)
   (if __forward_gain (setq _forward_gain __forward_gain)) _forward_gain)
  (:dead_band
   (&optional __dead_band)
   (if __dead_band (setq _dead_band __dead_band)) _dead_band)
  (:integral_pos_max
   (&optional __integral_pos_max)
   (if __integral_pos_max (setq _integral_pos_max __integral_pos_max)) _integral_pos_max)
  (:integral_neg_max
   (&optional __integral_neg_max)
   (if __integral_neg_max (setq _integral_neg_max __integral_neg_max)) _integral_neg_max)
  (:output_divider
   (&optional __output_divider)
   (if __output_divider (setq _output_divider __output_divider)) _output_divider)
  (:setpoint
   (&optional __setpoint)
   (if __setpoint (setq _setpoint __setpoint)) _setpoint)
  (:serialization-length
   ()
   (+
    ;; int32 _id
    4
    ;; uint8[] _motors
    (* 1    (length _motors)) 4
    ;; uint8[] _control_mode
    (* 1    (length _control_mode)) 4
    ;; int32[] _output_pos_max
    (* 4    (length _output_pos_max)) 4
    ;; int32[] _output_neg_max
    (* 4    (length _output_neg_max)) 4
    ;; int32[] _sp_pos_max
    (* 4    (length _sp_pos_max)) 4
    ;; int32[] _sp_neg_max
    (* 4    (length _sp_neg_max)) 4
    ;; uint16[] _kp
    (* 2    (length _kp)) 4
    ;; uint16[] _ki
    (* 2    (length _ki)) 4
    ;; uint16[] _kd
    (* 2    (length _kd)) 4
    ;; uint16[] _forward_gain
    (* 2    (length _forward_gain)) 4
    ;; uint16[] _dead_band
    (* 2    (length _dead_band)) 4
    ;; int32[] _integral_pos_max
    (* 4    (length _integral_pos_max)) 4
    ;; int32[] _integral_neg_max
    (* 4    (length _integral_neg_max)) 4
    ;; int32[] _output_divider
    (* 4    (length _output_divider)) 4
    ;; int32[] _setpoint
    (* 4    (length _setpoint)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _id
       (write-long _id s)
     ;; uint8[] _motors
     (write-long (length _motors) s)
     (princ _motors s)
     ;; uint8[] _control_mode
     (write-long (length _control_mode) s)
     (princ _control_mode s)
     ;; int32[] _output_pos_max
     (write-long (length _output_pos_max) s)
     (dotimes (i (length _output_pos_max))
       (write-long (elt _output_pos_max i) s)
       )
     ;; int32[] _output_neg_max
     (write-long (length _output_neg_max) s)
     (dotimes (i (length _output_neg_max))
       (write-long (elt _output_neg_max i) s)
       )
     ;; int32[] _sp_pos_max
     (write-long (length _sp_pos_max) s)
     (dotimes (i (length _sp_pos_max))
       (write-long (elt _sp_pos_max i) s)
       )
     ;; int32[] _sp_neg_max
     (write-long (length _sp_neg_max) s)
     (dotimes (i (length _sp_neg_max))
       (write-long (elt _sp_neg_max i) s)
       )
     ;; uint16[] _kp
     (write-long (length _kp) s)
     (dotimes (i (length _kp))
       (write-word (elt _kp i) s)
       )
     ;; uint16[] _ki
     (write-long (length _ki) s)
     (dotimes (i (length _ki))
       (write-word (elt _ki i) s)
       )
     ;; uint16[] _kd
     (write-long (length _kd) s)
     (dotimes (i (length _kd))
       (write-word (elt _kd i) s)
       )
     ;; uint16[] _forward_gain
     (write-long (length _forward_gain) s)
     (dotimes (i (length _forward_gain))
       (write-word (elt _forward_gain i) s)
       )
     ;; uint16[] _dead_band
     (write-long (length _dead_band) s)
     (dotimes (i (length _dead_band))
       (write-word (elt _dead_band i) s)
       )
     ;; int32[] _integral_pos_max
     (write-long (length _integral_pos_max) s)
     (dotimes (i (length _integral_pos_max))
       (write-long (elt _integral_pos_max i) s)
       )
     ;; int32[] _integral_neg_max
     (write-long (length _integral_neg_max) s)
     (dotimes (i (length _integral_neg_max))
       (write-long (elt _integral_neg_max i) s)
       )
     ;; int32[] _output_divider
     (write-long (length _output_divider) s)
     (dotimes (i (length _output_divider))
       (write-long (elt _output_divider i) s)
       )
     ;; int32[] _setpoint
     (write-long (length _setpoint) s)
     (dotimes (i (length _setpoint))
       (write-long (elt _setpoint i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _id
     (setq _id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint8[] _motors
   (let ((n (sys::peek buf ptr- :integer))) (incf ptr- 4)
     (setq _motors (make-array n :element-type :char))
     (replace _motors buf :start2 ptr-) (incf ptr- n))
   ;; uint8[] _control_mode
   (let ((n (sys::peek buf ptr- :integer))) (incf ptr- 4)
     (setq _control_mode (make-array n :element-type :char))
     (replace _control_mode buf :start2 ptr-) (incf ptr- n))
   ;; int32[] _output_pos_max
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _output_pos_max (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _output_pos_max i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; int32[] _output_neg_max
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _output_neg_max (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _output_neg_max i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; int32[] _sp_pos_max
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _sp_pos_max (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _sp_pos_max i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; int32[] _sp_neg_max
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _sp_neg_max (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _sp_neg_max i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; uint16[] _kp
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _kp (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _kp i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     ))
   ;; uint16[] _ki
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _ki (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _ki i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     ))
   ;; uint16[] _kd
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _kd (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _kd i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     ))
   ;; uint16[] _forward_gain
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _forward_gain (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _forward_gain i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     ))
   ;; uint16[] _dead_band
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _dead_band (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _dead_band i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     ))
   ;; int32[] _integral_pos_max
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _integral_pos_max (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _integral_pos_max i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; int32[] _integral_neg_max
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _integral_neg_max (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _integral_neg_max i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; int32[] _output_divider
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _output_divider (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _output_divider i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; int32[] _setpoint
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _setpoint (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _setpoint i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get roboy_middleware_msgs::MotorConfig :md5sum-) "c59b5188ed0d7529ac35ce108ce30bbc")
(setf (get roboy_middleware_msgs::MotorConfig :datatype-) "roboy_middleware_msgs/MotorConfig")
(setf (get roboy_middleware_msgs::MotorConfig :definition-)
      "int32 id
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
int32[] setpoint
")



(provide :roboy_middleware_msgs/MotorConfig "c59b5188ed0d7529ac35ce108ce30bbc")


