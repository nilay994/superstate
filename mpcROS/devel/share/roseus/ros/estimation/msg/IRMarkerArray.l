;; Auto-generated. Do not edit!


(when (boundp 'estimation::IRMarkerArray)
  (if (not (find-package "ESTIMATION"))
    (make-package "ESTIMATION"))
  (shadow 'IRMarkerArray (find-package "ESTIMATION")))
(unless (find-package "ESTIMATION::IRMARKERARRAY")
  (make-package "ESTIMATION::IRMARKERARRAY"))

(in-package "ROS")
;;//! \htmlinclude IRMarkerArray.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass estimation::IRMarkerArray
  :super ros::object
  :slots (_header _markers ))

(defmethod estimation::IRMarkerArray
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:markers __markers) (let (r) (dotimes (i 0) (push (instance estimation::IRMarker :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _markers __markers)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:markers
   (&rest __markers)
   (if (keywordp (car __markers))
       (send* _markers __markers)
     (progn
       (if __markers (setq _markers (car __markers)))
       _markers)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; estimation/IRMarker[] _markers
    (apply #'+ (send-all _markers :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; estimation/IRMarker[] _markers
     (write-long (length _markers) s)
     (dolist (elem _markers)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; estimation/IRMarker[] _markers
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _markers (let (r) (dotimes (i n) (push (instance estimation::IRMarker :init) r)) r))
     (dolist (elem- _markers)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get estimation::IRMarkerArray :md5sum-) "8091f0feface0cc0f973be1d2b5c29b5")
(setf (get estimation::IRMarkerArray :datatype-) "estimation/IRMarkerArray")
(setf (get estimation::IRMarkerArray :definition-)
      "# ROS Message that contains pixel-space IR marker locations and labels.

Header header
estimation/IRMarker[] markers

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: estimation/IRMarker
# ROS Message that contains pixel-space IR marker locations and labels.

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



(provide :estimation/IRMarkerArray "8091f0feface0cc0f973be1d2b5c29b5")


