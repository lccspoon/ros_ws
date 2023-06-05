;; Auto-generated. Do not edit!


(when (boundp 'traversability_msgs::TraversabilityResult)
  (if (not (find-package "TRAVERSABILITY_MSGS"))
    (make-package "TRAVERSABILITY_MSGS"))
  (shadow 'TraversabilityResult (find-package "TRAVERSABILITY_MSGS")))
(unless (find-package "TRAVERSABILITY_MSGS::TRAVERSABILITYRESULT")
  (make-package "TRAVERSABILITY_MSGS::TRAVERSABILITYRESULT"))

(in-package "ROS")
;;//! \htmlinclude TraversabilityResult.msg.html


(defclass traversability_msgs::TraversabilityResult
  :super ros::object
  :slots (_is_safe _traversability _area ))

(defmethod traversability_msgs::TraversabilityResult
  (:init
   (&key
    ((:is_safe __is_safe) nil)
    ((:traversability __traversability) 0.0)
    ((:area __area) 0.0)
    )
   (send-super :init)
   (setq _is_safe __is_safe)
   (setq _traversability (float __traversability))
   (setq _area (float __area))
   self)
  (:is_safe
   (&optional (__is_safe :null))
   (if (not (eq __is_safe :null)) (setq _is_safe __is_safe)) _is_safe)
  (:traversability
   (&optional __traversability)
   (if __traversability (setq _traversability __traversability)) _traversability)
  (:area
   (&optional __area)
   (if __area (setq _area __area)) _area)
  (:serialization-length
   ()
   (+
    ;; bool _is_safe
    1
    ;; float64 _traversability
    8
    ;; float64 _area
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _is_safe
       (if _is_safe (write-byte -1 s) (write-byte 0 s))
     ;; float64 _traversability
       (sys::poke _traversability (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _area
       (sys::poke _area (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _is_safe
     (setq _is_safe (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float64 _traversability
     (setq _traversability (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _area
     (setq _area (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get traversability_msgs::TraversabilityResult :md5sum-) "14ffe3323c91cd823bef7a313714954e")
(setf (get traversability_msgs::TraversabilityResult :datatype-) "traversability_msgs/TraversabilityResult")
(setf (get traversability_msgs::TraversabilityResult :definition-)
      "# If path is safe to traverse.
bool is_safe

# Estimate of the traversability of the path.
# Ranges from 0 to 1 where 0 means not traversable and 1 highly traversable.
float64 traversability

# Area of the footprint path.
float64 area
")



(provide :traversability_msgs/TraversabilityResult "14ffe3323c91cd823bef7a313714954e")


