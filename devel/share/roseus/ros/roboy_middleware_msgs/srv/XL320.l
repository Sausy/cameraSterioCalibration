;; Auto-generated. Do not edit!


(when (boundp 'roboy_middleware_msgs::XL320)
  (if (not (find-package "ROBOY_MIDDLEWARE_MSGS"))
    (make-package "ROBOY_MIDDLEWARE_MSGS"))
  (shadow 'XL320 (find-package "ROBOY_MIDDLEWARE_MSGS")))
(unless (find-package "ROBOY_MIDDLEWARE_MSGS::XL320")
  (make-package "ROBOY_MIDDLEWARE_MSGS::XL320"))
(unless (find-package "ROBOY_MIDDLEWARE_MSGS::XL320REQUEST")
  (make-package "ROBOY_MIDDLEWARE_MSGS::XL320REQUEST"))
(unless (find-package "ROBOY_MIDDLEWARE_MSGS::XL320RESPONSE")
  (make-package "ROBOY_MIDDLEWARE_MSGS::XL320RESPONSE"))

(in-package "ROS")





(defclass roboy_middleware_msgs::XL320Request
  :super ros::object
  :slots (_type _motor _address _value ))

(defmethod roboy_middleware_msgs::XL320Request
  (:init
   (&key
    ((:type __type) nil)
    ((:motor __motor) 0)
    ((:address __address) 0)
    ((:value __value) 0)
    )
   (send-super :init)
   (setq _type __type)
   (setq _motor (round __motor))
   (setq _address (round __address))
   (setq _value (round __value))
   self)
  (:type
   (&optional __type)
   (if __type (setq _type __type)) _type)
  (:motor
   (&optional __motor)
   (if __motor (setq _motor __motor)) _motor)
  (:address
   (&optional __address)
   (if __address (setq _address __address)) _address)
  (:value
   (&optional __value)
   (if __value (setq _value __value)) _value)
  (:serialization-length
   ()
   (+
    ;; bool _type
    1
    ;; uint8 _motor
    1
    ;; uint8 _address
    1
    ;; int16 _value
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _type
       (if _type (write-byte -1 s) (write-byte 0 s))
     ;; uint8 _motor
       (write-byte _motor s)
     ;; uint8 _address
       (write-byte _address s)
     ;; int16 _value
       (write-word _value s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _type
     (setq _type (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uint8 _motor
     (setq _motor (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _address
     (setq _address (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; int16 _value
     (setq _value (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(defclass roboy_middleware_msgs::XL320Response
  :super ros::object
  :slots (_value ))

(defmethod roboy_middleware_msgs::XL320Response
  (:init
   (&key
    ((:value __value) 0)
    )
   (send-super :init)
   (setq _value (round __value))
   self)
  (:value
   (&optional __value)
   (if __value (setq _value __value)) _value)
  (:serialization-length
   ()
   (+
    ;; int16 _value
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int16 _value
       (write-word _value s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int16 _value
     (setq _value (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(defclass roboy_middleware_msgs::XL320
  :super ros::object
  :slots ())

(setf (get roboy_middleware_msgs::XL320 :md5sum-) "2df206e3bbdc218ed0e86d48777a9fbf")
(setf (get roboy_middleware_msgs::XL320 :datatype-) "roboy_middleware_msgs/XL320")
(setf (get roboy_middleware_msgs::XL320 :request) roboy_middleware_msgs::XL320Request)
(setf (get roboy_middleware_msgs::XL320 :response) roboy_middleware_msgs::XL320Response)

(defmethod roboy_middleware_msgs::XL320Request
  (:response () (instance roboy_middleware_msgs::XL320Response :init)))

(setf (get roboy_middleware_msgs::XL320Request :md5sum-) "2df206e3bbdc218ed0e86d48777a9fbf")
(setf (get roboy_middleware_msgs::XL320Request :datatype-) "roboy_middleware_msgs/XL320Request")
(setf (get roboy_middleware_msgs::XL320Request :definition-)
      "
bool type
uint8 motor

































uint8 address
int16 value
---
int16 value
")

(setf (get roboy_middleware_msgs::XL320Response :md5sum-) "2df206e3bbdc218ed0e86d48777a9fbf")
(setf (get roboy_middleware_msgs::XL320Response :datatype-) "roboy_middleware_msgs/XL320Response")
(setf (get roboy_middleware_msgs::XL320Response :definition-)
      "
bool type
uint8 motor

































uint8 address
int16 value
---
int16 value
")



(provide :roboy_middleware_msgs/XL320 "2df206e3bbdc218ed0e86d48777a9fbf")


