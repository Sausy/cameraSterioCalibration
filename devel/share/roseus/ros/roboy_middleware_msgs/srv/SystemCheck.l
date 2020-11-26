;; Auto-generated. Do not edit!


(when (boundp 'roboy_middleware_msgs::SystemCheck)
  (if (not (find-package "ROBOY_MIDDLEWARE_MSGS"))
    (make-package "ROBOY_MIDDLEWARE_MSGS"))
  (shadow 'SystemCheck (find-package "ROBOY_MIDDLEWARE_MSGS")))
(unless (find-package "ROBOY_MIDDLEWARE_MSGS::SYSTEMCHECK")
  (make-package "ROBOY_MIDDLEWARE_MSGS::SYSTEMCHECK"))
(unless (find-package "ROBOY_MIDDLEWARE_MSGS::SYSTEMCHECKREQUEST")
  (make-package "ROBOY_MIDDLEWARE_MSGS::SYSTEMCHECKREQUEST"))
(unless (find-package "ROBOY_MIDDLEWARE_MSGS::SYSTEMCHECKRESPONSE")
  (make-package "ROBOY_MIDDLEWARE_MSGS::SYSTEMCHECKRESPONSE"))

(in-package "ROS")





(defclass roboy_middleware_msgs::SystemCheckRequest
  :super ros::object
  :slots (_motorids ))

(defmethod roboy_middleware_msgs::SystemCheckRequest
  (:init
   (&key
    ((:motorids __motorids) (make-array 0 :initial-element 0 :element-type :char))
    )
   (send-super :init)
   (setq _motorids __motorids)
   self)
  (:motorids
   (&optional __motorids)
   (if __motorids (setq _motorids __motorids)) _motorids)
  (:serialization-length
   ()
   (+
    ;; uint8[] _motorids
    (* 1    (length _motorids)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8[] _motorids
     (write-long (length _motorids) s)
     (princ _motorids s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8[] _motorids
   (let ((n (sys::peek buf ptr- :integer))) (incf ptr- 4)
     (setq _motorids (make-array n :element-type :char))
     (replace _motorids buf :start2 ptr-) (incf ptr- n))
   ;;
   self)
  )

(defclass roboy_middleware_msgs::SystemCheckResponse
  :super ros::object
  :slots (_position _displacement ))

(defmethod roboy_middleware_msgs::SystemCheckResponse
  (:init
   (&key
    ((:position __position) (let (r) (dotimes (i 0) (push nil r)) r))
    ((:displacement __displacement) (let (r) (dotimes (i 0) (push nil r)) r))
    )
   (send-super :init)
   (setq _position __position)
   (setq _displacement __displacement)
   self)
  (:position
   (&optional __position)
   (if __position (setq _position __position)) _position)
  (:displacement
   (&optional __displacement)
   (if __displacement (setq _displacement __displacement)) _displacement)
  (:serialization-length
   ()
   (+
    ;; bool[] _position
    (* 1    (length _position)) 4
    ;; bool[] _displacement
    (* 1    (length _displacement)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool[] _position
     (write-long (length _position) s)
     (dotimes (i (length _position))
       (if (elt _position i) (write-byte -1 s) (write-byte 0 s))
       )
     ;; bool[] _displacement
     (write-long (length _displacement) s)
     (dotimes (i (length _displacement))
       (if (elt _displacement i) (write-byte -1 s) (write-byte 0 s))
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool[] _position
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _position (make-list n))
     (dotimes (i n)
     (setf (elt _position i) (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
     ))
   ;; bool[] _displacement
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _displacement (make-list n))
     (dotimes (i n)
     (setf (elt _displacement i) (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
     ))
   ;;
   self)
  )

(defclass roboy_middleware_msgs::SystemCheck
  :super ros::object
  :slots ())

(setf (get roboy_middleware_msgs::SystemCheck :md5sum-) "d2bdc9a40e1feeae452cdda757d15ee0")
(setf (get roboy_middleware_msgs::SystemCheck :datatype-) "roboy_middleware_msgs/SystemCheck")
(setf (get roboy_middleware_msgs::SystemCheck :request) roboy_middleware_msgs::SystemCheckRequest)
(setf (get roboy_middleware_msgs::SystemCheck :response) roboy_middleware_msgs::SystemCheckResponse)

(defmethod roboy_middleware_msgs::SystemCheckRequest
  (:response () (instance roboy_middleware_msgs::SystemCheckResponse :init)))

(setf (get roboy_middleware_msgs::SystemCheckRequest :md5sum-) "d2bdc9a40e1feeae452cdda757d15ee0")
(setf (get roboy_middleware_msgs::SystemCheckRequest :datatype-) "roboy_middleware_msgs/SystemCheckRequest")
(setf (get roboy_middleware_msgs::SystemCheckRequest :definition-)
      "# if empty check all motors, otherwise only the requested
uint8[] motorids
---
# returns which control modes are functional
bool[] position
bool[] displacement
")

(setf (get roboy_middleware_msgs::SystemCheckResponse :md5sum-) "d2bdc9a40e1feeae452cdda757d15ee0")
(setf (get roboy_middleware_msgs::SystemCheckResponse :datatype-) "roboy_middleware_msgs/SystemCheckResponse")
(setf (get roboy_middleware_msgs::SystemCheckResponse :definition-)
      "# if empty check all motors, otherwise only the requested
uint8[] motorids
---
# returns which control modes are functional
bool[] position
bool[] displacement
")



(provide :roboy_middleware_msgs/SystemCheck "d2bdc9a40e1feeae452cdda757d15ee0")


