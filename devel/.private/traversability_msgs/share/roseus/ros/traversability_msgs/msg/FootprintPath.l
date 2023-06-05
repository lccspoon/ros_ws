;; Auto-generated. Do not edit!


(when (boundp 'traversability_msgs::FootprintPath)
  (if (not (find-package "TRAVERSABILITY_MSGS"))
    (make-package "TRAVERSABILITY_MSGS"))
  (shadow 'FootprintPath (find-package "TRAVERSABILITY_MSGS")))
(unless (find-package "TRAVERSABILITY_MSGS::FOOTPRINTPATH")
  (make-package "TRAVERSABILITY_MSGS::FOOTPRINTPATH"))

(in-package "ROS")
;;//! \htmlinclude FootprintPath.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass traversability_msgs::FootprintPath
  :super ros::object
  :slots (_poses _radius _footprint _conservative _compute_untraversable_polygon ))

(defmethod traversability_msgs::FootprintPath
  (:init
   (&key
    ((:poses __poses) (instance geometry_msgs::PoseArray :init))
    ((:radius __radius) 0.0)
    ((:footprint __footprint) (instance geometry_msgs::PolygonStamped :init))
    ((:conservative __conservative) nil)
    ((:compute_untraversable_polygon __compute_untraversable_polygon) nil)
    )
   (send-super :init)
   (setq _poses __poses)
   (setq _radius (float __radius))
   (setq _footprint __footprint)
   (setq _conservative __conservative)
   (setq _compute_untraversable_polygon __compute_untraversable_polygon)
   self)
  (:poses
   (&rest __poses)
   (if (keywordp (car __poses))
       (send* _poses __poses)
     (progn
       (if __poses (setq _poses (car __poses)))
       _poses)))
  (:radius
   (&optional __radius)
   (if __radius (setq _radius __radius)) _radius)
  (:footprint
   (&rest __footprint)
   (if (keywordp (car __footprint))
       (send* _footprint __footprint)
     (progn
       (if __footprint (setq _footprint (car __footprint)))
       _footprint)))
  (:conservative
   (&optional (__conservative :null))
   (if (not (eq __conservative :null)) (setq _conservative __conservative)) _conservative)
  (:compute_untraversable_polygon
   (&optional (__compute_untraversable_polygon :null))
   (if (not (eq __compute_untraversable_polygon :null)) (setq _compute_untraversable_polygon __compute_untraversable_polygon)) _compute_untraversable_polygon)
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/PoseArray _poses
    (send _poses :serialization-length)
    ;; float64 _radius
    8
    ;; geometry_msgs/PolygonStamped _footprint
    (send _footprint :serialization-length)
    ;; bool _conservative
    1
    ;; bool _compute_untraversable_polygon
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/PoseArray _poses
       (send _poses :serialize s)
     ;; float64 _radius
       (sys::poke _radius (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; geometry_msgs/PolygonStamped _footprint
       (send _footprint :serialize s)
     ;; bool _conservative
       (if _conservative (write-byte -1 s) (write-byte 0 s))
     ;; bool _compute_untraversable_polygon
       (if _compute_untraversable_polygon (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/PoseArray _poses
     (send _poses :deserialize buf ptr-) (incf ptr- (send _poses :serialization-length))
   ;; float64 _radius
     (setq _radius (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; geometry_msgs/PolygonStamped _footprint
     (send _footprint :deserialize buf ptr-) (incf ptr- (send _footprint :serialization-length))
   ;; bool _conservative
     (setq _conservative (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _compute_untraversable_polygon
     (setq _compute_untraversable_polygon (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get traversability_msgs::FootprintPath :md5sum-) "6a088c23f98a81d850097dff6ef7145d")
(setf (get traversability_msgs::FootprintPath :datatype-) "traversability_msgs/FootprintPath")
(setf (get traversability_msgs::FootprintPath :definition-)
      "# Path of the footprint defined as array of connected poses.
# Poses are connected piece-wise linear in the order of the array.
geometry_msgs/PoseArray poses

# Either: Define footprint radius.
float64 radius 

# Or: Define footprint as polygon in the robot base frame.
# Polygon is used if it is defined, otherwise radius is used.
geometry_msgs/PolygonStamped footprint

# Use conservative footprint. Only available if polygon is used.
bool conservative

# Compute untraversable polygon in the checked area for traversability. If true, computation demand is higher.
bool compute_untraversable_polygon

================================================================================
MSG: geometry_msgs/PoseArray
# An array of poses with a header for global reference.

Header header

Pose[] poses

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
string frame_id

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: geometry_msgs/PolygonStamped
# This represents a Polygon with reference coordinate frame and timestamp
Header header
Polygon polygon

================================================================================
MSG: geometry_msgs/Polygon
#A specification of a polygon where the first and last points are assumed to be connected
Point32[] points

================================================================================
MSG: geometry_msgs/Point32
# This contains the position of a point in free space(with 32 bits of precision).
# It is recommeded to use Point wherever possible instead of Point32.  
# 
# This recommendation is to promote interoperability.  
#
# This message is designed to take up less space when sending
# lots of points at once, as in the case of a PointCloud.  

float32 x
float32 y
float32 z
")



(provide :traversability_msgs/FootprintPath "6a088c23f98a81d850097dff6ef7145d")


