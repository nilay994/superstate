; Auto-generated. Do not edit!


(cl:in-package estimation-msg)


;//! \htmlinclude IRMarker.msg.html

(cl:defclass <IRMarker> (roslisp-msg-protocol:ros-message)
  ((landmarkID
    :reader landmarkID
    :initarg :landmarkID
    :type std_msgs-msg:String
    :initform (cl:make-instance 'std_msgs-msg:String))
   (markerID
    :reader markerID
    :initarg :markerID
    :type std_msgs-msg:String
    :initform (cl:make-instance 'std_msgs-msg:String))
   (x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0)
   (z
    :reader z
    :initarg :z
    :type cl:float
    :initform 0.0))
)

(cl:defclass IRMarker (<IRMarker>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <IRMarker>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'IRMarker)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name estimation-msg:<IRMarker> is deprecated: use estimation-msg:IRMarker instead.")))

(cl:ensure-generic-function 'landmarkID-val :lambda-list '(m))
(cl:defmethod landmarkID-val ((m <IRMarker>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader estimation-msg:landmarkID-val is deprecated.  Use estimation-msg:landmarkID instead.")
  (landmarkID m))

(cl:ensure-generic-function 'markerID-val :lambda-list '(m))
(cl:defmethod markerID-val ((m <IRMarker>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader estimation-msg:markerID-val is deprecated.  Use estimation-msg:markerID instead.")
  (markerID m))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <IRMarker>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader estimation-msg:x-val is deprecated.  Use estimation-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <IRMarker>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader estimation-msg:y-val is deprecated.  Use estimation-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'z-val :lambda-list '(m))
(cl:defmethod z-val ((m <IRMarker>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader estimation-msg:z-val is deprecated.  Use estimation-msg:z instead.")
  (z m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <IRMarker>) ostream)
  "Serializes a message object of type '<IRMarker>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'landmarkID) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'markerID) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <IRMarker>) istream)
  "Deserializes a message object of type '<IRMarker>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'landmarkID) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'markerID) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'z) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<IRMarker>)))
  "Returns string type for a message object of type '<IRMarker>"
  "estimation/IRMarker")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'IRMarker)))
  "Returns string type for a message object of type 'IRMarker"
  "estimation/IRMarker")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<IRMarker>)))
  "Returns md5sum for a message object of type '<IRMarker>"
  "74186740d14e922a7e5ca083a6795f31")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'IRMarker)))
  "Returns md5sum for a message object of type 'IRMarker"
  "74186740d14e922a7e5ca083a6795f31")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<IRMarker>)))
  "Returns full string definition for message of type '<IRMarker>"
  (cl:format cl:nil "# ROS Message that contains pixel-space IR marker locations and labels.~%~%# ID of the landmark (e.g. gate) that the IR marker is attached to~%std_msgs/String landmarkID~%# ID of individual marker.~%std_msgs/String markerID~%~%float32 x~%float32 y~%# Z is the distance from the camera in meters.~%float32 z~%================================================================================~%MSG: std_msgs/String~%string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'IRMarker)))
  "Returns full string definition for message of type 'IRMarker"
  (cl:format cl:nil "# ROS Message that contains pixel-space IR marker locations and labels.~%~%# ID of the landmark (e.g. gate) that the IR marker is attached to~%std_msgs/String landmarkID~%# ID of individual marker.~%std_msgs/String markerID~%~%float32 x~%float32 y~%# Z is the distance from the camera in meters.~%float32 z~%================================================================================~%MSG: std_msgs/String~%string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <IRMarker>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'landmarkID))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'markerID))
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <IRMarker>))
  "Converts a ROS message object to a list"
  (cl:list 'IRMarker
    (cl:cons ':landmarkID (landmarkID msg))
    (cl:cons ':markerID (markerID msg))
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':z (z msg))
))
