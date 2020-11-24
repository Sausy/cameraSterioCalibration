;; Auto-generated. Do not edit!


(when (boundp 'roboy_middleware_msgs::DarkRoomSensorV2)
  (if (not (find-package "ROBOY_MIDDLEWARE_MSGS"))
    (make-package "ROBOY_MIDDLEWARE_MSGS"))
  (shadow 'DarkRoomSensorV2 (find-package "ROBOY_MIDDLEWARE_MSGS")))
(unless (find-package "ROBOY_MIDDLEWARE_MSGS::DARKROOMSENSORV2")
  (make-package "ROBOY_MIDDLEWARE_MSGS::DARKROOMSENSORV2"))

(in-package "ROS")
;;//! \htmlinclude DarkRoomSensorV2.msg.html


(defclass roboy_middleware_msgs::DarkRoomSensorV2
  :super ros::object
  :slots (_object_id _base _SensorID _elevation _azimuth ))

(defmethod roboy_middleware_msgs::DarkRoomSensorV2
  (:init
   (&key
    ((:object_id __object_id) "")
    ((:base __base) 0)
    ((:SensorID __SensorID) 0)
    ((:elevation __elevation) 0.0)
    ((:azimuth __azimuth) 0.0)
    )
   (send-super :init)
   (setq _object_id (string __object_id))
   (setq _base (round __base))
   (setq _SensorID (round __SensorID))
   (setq _elevation (float __elevation))
   (setq _azimuth (float __azimuth))
   self)
  (:object_id
   (&optional __object_id)
   (if __object_id (setq _object_id __object_id)) _object_id)
  (:base
   (&optional __base)
   (if __base (setq _base __base)) _base)
  (:SensorID
   (&optional __SensorID)
   (if __SensorID (setq _SensorID __SensorID)) _SensorID)
  (:elevation
   (&optional __elevation)
   (if __elevation (setq _elevation __elevation)) _elevation)
  (:azimuth
   (&optional __azimuth)
   (if __azimuth (setq _azimuth __azimuth)) _azimuth)
  (:serialization-length
   ()
   (+
    ;; string _object_id
    4 (length _object_id)
    ;; int8 _base
    1
    ;; int8 _SensorID
    1
    ;; float64 _elevation
    8
    ;; float64 _azimuth
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _object_id
       (write-long (length _object_id) s) (princ _object_id s)
     ;; int8 _base
       (write-byte _base s)
     ;; int8 _SensorID
       (write-byte _SensorID s)
     ;; float64 _elevation
       (sys::poke _elevation (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _azimuth
       (sys::poke _azimuth (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _object_id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _object_id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int8 _base
     (setq _base (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _base 127) (setq _base (- _base 256)))
   ;; int8 _SensorID
     (setq _SensorID (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _SensorID 127) (setq _SensorID (- _SensorID 256)))
   ;; float64 _elevation
     (setq _elevation (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _azimuth
     (setq _azimuth (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get roboy_middleware_msgs::DarkRoomSensorV2 :md5sum-) "33742585e0d9d4936d60498b07ad60de")
(setf (get roboy_middleware_msgs::DarkRoomSensorV2 :datatype-) "roboy_middleware_msgs/DarkRoomSensorV2")
(setf (get roboy_middleware_msgs::DarkRoomSensorV2 :definition-)
      "string object_id
int8 base
int8 SensorID
float64 elevation
float64 azimuth

")



(provide :roboy_middleware_msgs/DarkRoomSensorV2 "33742585e0d9d4936d60498b07ad60de")


