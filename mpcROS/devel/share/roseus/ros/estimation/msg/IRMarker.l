;; Auto-generated. Do not edit!


(when (boundp 'estimation::IRMarker)
  (if (not (find-package "ESTIMATION"))
    (make-package "ESTIMATION"))
  (shadow 'IRMarker (find-package "ESTIMATION")))
(unless (find-package "ESTIMATION::IRMARKER")
  (make-package "ESTIMATION::IRMARKER"))

(in-package "ROS")
;;//! \htmlinclude IRMarker.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass estimation::IRMarker
  :super ros::object
  :slots (_landmarkID _markerID _x _y _z ))

(defmethod estimation::IRMarker
  (:init
   (&key
    ((:landmarkID __landmarkID) (instance std_msgs::String :init))
    ((:markerID __markerID) (instance std_msgs::String :init))
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:z __z) 0.0)
    )
   (send-super :init)
   (setq _landmarkID __landmarkID)
   (setq _markerID __markerID)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _z (float __z))
   self)
  (:landmarkID
   (&rest __landmarkID)
   (if (keywordp (car __landmarkID))
       (send* _landmarkID __landmarkID)
     (progn
       (if __landmarkID (setq _landmarkID (car __landmarkID)))
       _landmarkID)))
  (:markerID
   (&rest __markerID)
   (if (keywordp (car __markerID))
       (send* _markerID __markerID)
     (progn
       (if __markerID (setq _markerID (car __markerID)))
       _markerID)))
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
    ;; std_msgs/String _landmarkID
    (send _landmarkID :serialization-length)
    ;; std_msgs/String _markerID
    (send _markerID :serialization-length)
    ;; float32 _x
    4
    ;; float32 _y
    4
    ;; float32 _z
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/String _landmarkID
       (send _landmarkID :serialize s)
     ;; std_msgs/String _markerID
       (send _markerID :serialize s)
     ;; float32 _x
       (sys::poke _x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y
       (sys::poke _y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _z
       (sys::poke _z (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/String _landmarkID
     (send _landmarkID :deserialize buf ptr-) (incf ptr- (send _landmarkID :serialization-length))
   ;; std_msgs/String _markerID
     (send _markerID :deserialize buf ptr-) (incf ptr- (send _markerID :serialization-length))
   ;; float32 _x
     (setq _x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y
     (setq _y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _z
     (setq _z (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get estimation::IRMarker :md5sum-) "74186740d14e922a7e5ca083a6795f31")
(setf (get estimation::IRMarker :datatype-) "estimation/IRMarker")
(setf (get estimation::IRMarker :definition-)
      "# ROS Message that contains pixel-space IR marker locations and labels.

# ID of the landmark (e.g. gate) that the IR marker is attached to
std_msgs/String landmarkID
# ID of individual marker.
std_msgs/String markerID

float32 x
float32 y
# Z is the distance from the camera in meters.
float32 z
================================================================================
MSG: std_msgs/String
string data

")



(provide :estimation/IRMarker "74186740d14e922a7e5ca083a6795f31")


