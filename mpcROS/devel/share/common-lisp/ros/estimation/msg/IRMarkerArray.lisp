; Auto-generated. Do not edit!


(cl:in-package estimation-msg)


;//! \htmlinclude IRMarkerArray.msg.html

(cl:defclass <IRMarkerArray> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (markers
    :reader markers
    :initarg :markers
    :type (cl:vector estimation-msg:IRMarker)
   :initform (cl:make-array 0 :element-type 'estimation-msg:IRMarker :initial-element (cl:make-instance 'estimation-msg:IRMarker))))
)

(cl:defclass IRMarkerArray (<IRMarkerArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <IRMarkerArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'IRMarkerArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name estimation-msg:<IRMarkerArray> is deprecated: use estimation-msg:IRMarkerArray instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <IRMarkerArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader estimation-msg:header-val is deprecated.  Use estimation-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'markers-val :lambda-list '(m))
(cl:defmethod markers-val ((m <IRMarkerArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader estimation-msg:markers-val is deprecated.  Use estimation-msg:markers instead.")
  (markers m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <IRMarkerArray>) ostream)
  "Serializes a message object of type '<IRMarkerArray>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'markers))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'markers))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <IRMarkerArray>) istream)
  "Deserializes a message object of type '<IRMarkerArray>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'markers) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'markers)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'estimation-msg:IRMarker))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<IRMarkerArray>)))
  "Returns string type for a message object of type '<IRMarkerArray>"
  "estimation/IRMarkerArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'IRMarkerArray)))
  "Returns string type for a message object of type 'IRMarkerArray"
  "estimation/IRMarkerArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<IRMarkerArray>)))
  "Returns md5sum for a message object of type '<IRMarkerArray>"
  "8091f0feface0cc0f973be1d2b5c29b5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'IRMarkerArray)))
  "Returns md5sum for a message object of type 'IRMarkerArray"
  "8091f0feface0cc0f973be1d2b5c29b5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<IRMarkerArray>)))
  "Returns full string definition for message of type '<IRMarkerArray>"
  (cl:format cl:nil "# ROS Message that contains pixel-space IR marker locations and labels.~%~%Header header~%estimation/IRMarker[] markers~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: estimation/IRMarker~%# ROS Message that contains pixel-space IR marker locations and labels.~%~%# ID of the landmark (e.g. gate) that the IR marker is attached to~%std_msgs/String landmarkID~%# ID of individual marker.~%std_msgs/String markerID~%~%float32 x~%float32 y~%# Z is the distance from the camera in meters.~%float32 z~%================================================================================~%MSG: std_msgs/String~%string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'IRMarkerArray)))
  "Returns full string definition for message of type 'IRMarkerArray"
  (cl:format cl:nil "# ROS Message that contains pixel-space IR marker locations and labels.~%~%Header header~%estimation/IRMarker[] markers~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: estimation/IRMarker~%# ROS Message that contains pixel-space IR marker locations and labels.~%~%# ID of the landmark (e.g. gate) that the IR marker is attached to~%std_msgs/String landmarkID~%# ID of individual marker.~%std_msgs/String markerID~%~%float32 x~%float32 y~%# Z is the distance from the camera in meters.~%float32 z~%================================================================================~%MSG: std_msgs/String~%string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <IRMarkerArray>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'markers) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <IRMarkerArray>))
  "Converts a ROS message object to a list"
  (cl:list 'IRMarkerArray
    (cl:cons ':header (header msg))
    (cl:cons ':markers (markers msg))
))
