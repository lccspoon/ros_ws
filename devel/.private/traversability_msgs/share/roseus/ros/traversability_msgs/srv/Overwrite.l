;; Auto-generated. Do not edit!


(when (boundp 'traversability_msgs::Overwrite)
  (if (not (find-package "TRAVERSABILITY_MSGS"))
    (make-package "TRAVERSABILITY_MSGS"))
  (shadow 'Overwrite (find-package "TRAVERSABILITY_MSGS")))
(unless (find-package "TRAVERSABILITY_MSGS::OVERWRITE")
  (make-package "TRAVERSABILITY_MSGS::OVERWRITE"))
(unless (find-package "TRAVERSABILITY_MSGS::OVERWRITEREQUEST")
  (make-package "TRAVERSABILITY_MSGS::OVERWRITEREQUEST"))
(unless (find-package "TRAVERSABILITY_MSGS::OVERWRITERESPONSE")
  (make-package "TRAVERSABILITY_MSGS::OVERWRITERESPONSE"))

(in-package "ROS")





(defclass traversability_msgs::OverwriteRequest
  :super ros::object
  :slots (_enable ))

(defmethod traversability_msgs::OverwriteRequest
  (:init
   (&key
    ((:enable __enable) nil)
    )
   (send-super :init)
   (setq _enable __enable)
   self)
  (:enable
   (&optional (__enable :null))
   (if (not (eq __enable :null)) (setq _enable __enable)) _enable)
  (:serialization-length
   ()
   (+
    ;; bool _enable
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _enable
       (if _enable (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _enable
     (setq _enable (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass traversability_msgs::OverwriteResponse
  :super ros::object
  :slots ())

(defmethod traversability_msgs::OverwriteResponse
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

(defclass traversability_msgs::Overwrite
  :super ros::object
  :slots ())

(setf (get traversability_msgs::Overwrite :md5sum-) "8c1211af706069c994c06e00eb59ac9e")
(setf (get traversability_msgs::Overwrite :datatype-) "traversability_msgs/Overwrite")
(setf (get traversability_msgs::Overwrite :request) traversability_msgs::OverwriteRequest)
(setf (get traversability_msgs::Overwrite :response) traversability_msgs::OverwriteResponse)

(defmethod traversability_msgs::OverwriteRequest
  (:response () (instance traversability_msgs::OverwriteResponse :init)))

(setf (get traversability_msgs::OverwriteRequest :md5sum-) "8c1211af706069c994c06e00eb59ac9e")
(setf (get traversability_msgs::OverwriteRequest :datatype-) "traversability_msgs/OverwriteRequest")
(setf (get traversability_msgs::OverwriteRequest :definition-)
      "# True to enable, false for disable.
bool enable
---

")

(setf (get traversability_msgs::OverwriteResponse :md5sum-) "8c1211af706069c994c06e00eb59ac9e")
(setf (get traversability_msgs::OverwriteResponse :datatype-) "traversability_msgs/OverwriteResponse")
(setf (get traversability_msgs::OverwriteResponse :definition-)
      "# True to enable, false for disable.
bool enable
---

")



(provide :traversability_msgs/Overwrite "8c1211af706069c994c06e00eb59ac9e")


