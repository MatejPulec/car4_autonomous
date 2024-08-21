;; Auto-generated. Do not edit!


(when (boundp 'odometry::CarState)
  (if (not (find-package "ODOMETRY"))
    (make-package "ODOMETRY"))
  (shadow 'CarState (find-package "ODOMETRY")))
(unless (find-package "ODOMETRY::CARSTATE")
  (make-package "ODOMETRY::CARSTATE"))

(in-package "ROS")
;;//! \htmlinclude CarState.msg.html


(defclass odometry::CarState
  :super ros::object
  :slots (_x _y _angle ))

(defmethod odometry::CarState
  (:init
   (&key
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:angle __angle) 0.0)
    )
   (send-super :init)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _angle (float __angle))
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:angle
   (&optional __angle)
   (if __angle (setq _angle __angle)) _angle)
  (:serialization-length
   ()
   (+
    ;; float32 _x
    4
    ;; float32 _y
    4
    ;; float32 _angle
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _x
       (sys::poke _x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y
       (sys::poke _y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _angle
       (sys::poke _angle (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _x
     (setq _x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y
     (setq _y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _angle
     (setq _angle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get odometry::CarState :md5sum-) "39617ea5ffa910b78cdf07b659b77ce4")
(setf (get odometry::CarState :datatype-) "odometry/CarState")
(setf (get odometry::CarState :definition-)
      "float32 x
float32 y
float32 angle
")



(provide :odometry/CarState "39617ea5ffa910b78cdf07b659b77ce4")


