
(cl:in-package :asdf)

(defsystem "traversability_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "FootprintPath" :depends-on ("_package_FootprintPath"))
    (:file "_package_FootprintPath" :depends-on ("_package"))
    (:file "TraversabilityResult" :depends-on ("_package_TraversabilityResult"))
    (:file "_package_TraversabilityResult" :depends-on ("_package"))
  ))