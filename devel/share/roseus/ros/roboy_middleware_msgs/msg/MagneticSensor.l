;; Auto-generated. Do not edit!


(when (boundp 'roboy_middleware_msgs::MagneticSensor)
  (if (not (find-package "ROBOY_MIDDLEWARE_MSGS"))
    (make-package "ROBOY_MIDDLEWARE_MSGS"))
  (shadow 'MagneticSensor (find-package "ROBOY_MIDDLEWARE_MSGS")))
(unless (find-package "ROBOY_MIDDLEWARE_MSGS::MAGNETICSENSOR")
  (make-package "ROBOY_MIDDLEWARE_MSGS::MAGNETICSENSOR"))

(in-package "ROS")
;;//! \htmlinclude MagneticSensor.msg.html


(defclass roboy_middleware_msgs::MagneticSensor
  :super ros::object
  :slots (_id _sensor_id _x _y _z ))

(defmethod roboy_middleware_msgs::MagneticSensor
  (:init
   (&key
    ((:id __id) 0)
    ((:sensor_id __sensor_id) (make-array 0 :initial-element 0 :element-type :char))
    ((:x __x) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:y __y) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:z __z) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _id (round __id))
   (setq _sensor_id __sensor_id)
   (setq _x __x)
   (setq _y __y)
   (setq _z __z)
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:sensor_id
   (&optional __sensor_id)
   (if __sensor_id (setq _sensor_id __sensor_id)) _sensor_id)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:z
   (&optional __z)
   (if __z (setq _z __z)) _z)
  (:serialization-length
   ()
   (+
    ;; uint8 _id
    1
    ;; uint8[] _sensor_id
    (* 1    (length _sensor_id)) 4
    ;; float32[] _x
    (* 4    (length _x)) 4
    ;; float32[] _y
    (* 4    (length _y)) 4
    ;; float32[] _z
    (* 4    (length _z)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _id
       (write-byte _id s)
     ;; uint8[] _sensor_id
     (write-long (length _sensor_id) s)
     (princ _sensor_id s)
     ;; float32[] _x
     (write-long (length _x) s)
     (dotimes (i (length _x))
       (sys::poke (elt _x i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _y
     (write-long (length _y) s)
     (dotimes (i (length _y))
       (sys::poke (elt _y i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _z
     (write-long (length _z) s)
     (dotimes (i (length _z))
       (sys::poke (elt _z i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _id
     (setq _id (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8[] _sensor_id
   (let ((n (sys::peek buf ptr- :integer))) (incf ptr- 4)
     (setq _sensor_id (make-array n :element-type :char))
     (replace _sensor_id buf :start2 ptr-) (incf ptr- n))
   ;; float32[] _x
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _x (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _x i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _y
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _y (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _y i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _z
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _z (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _z i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get roboy_middleware_msgs::MagneticSensor :md5sum-) "6ece4c3827719acb61cc1cbe83060dc5")
(setf (get roboy_middleware_msgs::MagneticSensor :datatype-) "roboy_middleware_msgs/MagneticSensor")
(setf (get roboy_middleware_msgs::MagneticSensor :definition-)
      "uint8 id
uint8[] sensor_id
float32[] x
float32[] y
float32[] z
")



(provide :roboy_middleware_msgs/MagneticSensor "6ece4c3827719acb61cc1cbe83060dc5")


