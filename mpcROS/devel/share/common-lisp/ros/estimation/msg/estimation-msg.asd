
(cl:in-package :asdf)

(defsystem "estimation-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "IRMarker" :depends-on ("_package_IRMarker"))
    (:file "_package_IRMarker" :depends-on ("_package"))
    (:file "IRMarkerArray" :depends-on ("_package_IRMarkerArray"))
    (:file "_package_IRMarkerArray" :depends-on ("_package"))
  ))