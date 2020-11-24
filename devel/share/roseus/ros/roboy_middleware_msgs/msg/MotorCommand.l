;; Auto-generated. Do not edit!


(when (boundp 'roboy_middleware_msgs::MotorCommand)
  (if (not (find-package "ROBOY_MIDDLEWARE_MSGS"))
    (make-package "ROBOY_MIDDLEWARE_MSGS"))
  (shadow 'MotorCommand (find-package "ROBOY_MIDDLEWARE_MSGS")))
(unless (find-package "ROBOY_MIDDLEWARE_MSGS::MOTORCOMMAND")
  (make-package "ROBOY_MIDDLEWARE_MSGS::MOTORCOMMAND"))

(in-package "ROS")
;;//! \htmlinclude MotorCommand.msg.html


(defclass roboy_middleware_msgs::MotorCommand
  :super ros::object
  :slots (_id _motors _set_points ))

(defmethod roboy_middleware_msgs::MotorCommand
  (:init
   (&key
    ((:id __id) 0)
    ((:motors __motors) (make-array 0 :initial-element 0 :element-type :integer))
    ((:set_points __set_points) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _id (round __id))
   (setq _motors __motors)
   (setq _set_points __set_points)
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:motors
   (&optional __motors)
   (if __motors (setq _motors __motors)) _motors)
  (:set_points
   (&optional __set_points)
   (if __set_points (setq _set_points __set_points)) _set_points)
  (:serialization-length
   ()
   (+
    ;; uint8 _id
    1
    ;; int32[] _motors
    (* 4    (length _motors)) 4
    ;; float32[] _set_points
    (* 4    (length _set_points)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _id
       (write-byte _id s)
     ;; int32[] _motors
     (write-long (length _motors) s)
     (dotimes (i (length _motors))
       (write-long (elt _motors i) s)
       )
     ;; float32[] _set_points
     (write-long (length _set_points) s)
     (dotimes (i (length _set_points))
       (sys::poke (elt _set_points i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _id
     (setq _id (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; int32[] _motors
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _motors (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _motors i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; float32[] _set_points
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _set_points (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _set_points i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get roboy_middleware_msgs::MotorCommand :md5sum-) "39efb397cd93ab4cb00018c2fb0a7c3c")
(setf (get roboy_middleware_msgs::MotorCommand :datatype-) "roboy_middleware_msgs/MotorCommand")
(setf (get roboy_middleware_msgs::MotorCommand :definition-)
      "uint8 id
int32[] motors
float32[] set_points

")



(provide :roboy_middleware_msgs/MotorCommand "39efb397cd93ab4cb00018c2fb0a7c3c")


