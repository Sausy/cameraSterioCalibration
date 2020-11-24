;; Auto-generated. Do not edit!


(when (boundp 'roboy_middleware_msgs::DanceTrajectory)
  (if (not (find-package "ROBOY_MIDDLEWARE_MSGS"))
    (make-package "ROBOY_MIDDLEWARE_MSGS"))
  (shadow 'DanceTrajectory (find-package "ROBOY_MIDDLEWARE_MSGS")))
(unless (find-package "ROBOY_MIDDLEWARE_MSGS::DANCETRAJECTORY")
  (make-package "ROBOY_MIDDLEWARE_MSGS::DANCETRAJECTORY"))
(unless (find-package "ROBOY_MIDDLEWARE_MSGS::DANCETRAJECTORYREQUEST")
  (make-package "ROBOY_MIDDLEWARE_MSGS::DANCETRAJECTORYREQUEST"))
(unless (find-package "ROBOY_MIDDLEWARE_MSGS::DANCETRAJECTORYRESPONSE")
  (make-package "ROBOY_MIDDLEWARE_MSGS::DANCETRAJECTORYRESPONSE"))

(in-package "ROS")





(defclass roboy_middleware_msgs::DanceTrajectoryRequest
  :super ros::object
  :slots (_trajectory_name ))

(defmethod roboy_middleware_msgs::DanceTrajectoryRequest
  (:init
   (&key
    ((:trajectory_name __trajectory_name) "")
    )
   (send-super :init)
   (setq _trajectory_name (string __trajectory_name))
   self)
  (:trajectory_name
   (&optional __trajectory_name)
   (if __trajectory_name (setq _trajectory_name __trajectory_name)) _trajectory_name)
  (:serialization-length
   ()
   (+
    ;; string _trajectory_name
    4 (length _trajectory_name)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _trajectory_name
       (write-long (length _trajectory_name) s) (princ _trajectory_name s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _trajectory_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _trajectory_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass roboy_middleware_msgs::DanceTrajectoryResponse
  :super ros::object
  :slots ())

(defmethod roboy_middleware_msgs::DanceTrajectoryResponse
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass roboy_middleware_msgs::DanceTrajectory
  :super ros::object
  :slots ())

(setf (get roboy_middleware_msgs::DanceTrajectory :md5sum-) "c1811518113b89edafd5bab82a308c0b")
(setf (get roboy_middleware_msgs::DanceTrajectory :datatype-) "roboy_middleware_msgs/DanceTrajectory")
(setf (get roboy_middleware_msgs::DanceTrajectory :request) roboy_middleware_msgs::DanceTrajectoryRequest)
(setf (get roboy_middleware_msgs::DanceTrajectory :response) roboy_middleware_msgs::DanceTrajectoryResponse)

(defmethod roboy_middleware_msgs::DanceTrajectoryRequest
  (:response () (instance roboy_middleware_msgs::DanceTrajectoryResponse :init)))

(setf (get roboy_middleware_msgs::DanceTrajectoryRequest :md5sum-) "c1811518113b89edafd5bab82a308c0b")
(setf (get roboy_middleware_msgs::DanceTrajectoryRequest :datatype-) "roboy_middleware_msgs/DanceTrajectoryRequest")
(setf (get roboy_middleware_msgs::DanceTrajectoryRequest :definition-)
      "string trajectory_name
---

")

(setf (get roboy_middleware_msgs::DanceTrajectoryResponse :md5sum-) "c1811518113b89edafd5bab82a308c0b")
(setf (get roboy_middleware_msgs::DanceTrajectoryResponse :datatype-) "roboy_middleware_msgs/DanceTrajectoryResponse")
(setf (get roboy_middleware_msgs::DanceTrajectoryResponse :definition-)
      "string trajectory_name
---

")



(provide :roboy_middleware_msgs/DanceTrajectory "c1811518113b89edafd5bab82a308c0b")


