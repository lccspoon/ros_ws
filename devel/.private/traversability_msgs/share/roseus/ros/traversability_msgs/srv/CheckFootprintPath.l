;; Auto-generated. Do not edit!


(when (boundp 'traversability_msgs::CheckFootprintPath)
  (if (not (find-package "TRAVERSABILITY_MSGS"))
    (make-package "TRAVERSABILITY_MSGS"))
  (shadow 'CheckFootprintPath (find-package "TRAVERSABILITY_MSGS")))
(unless (find-package "TRAVERSABILITY_MSGS::CHECKFOOTPRINTPATH")
  (make-package "TRAVERSABILITY_MSGS::CHECKFOOTPRINTPATH"))
(unless (find-package "TRAVERSABILITY_MSGS::CHECKFOOTPRINTPATHREQUEST")
  (make-package "TRAVERSABILITY_MSGS::CHECKFOOTPRINTPATHREQUEST"))
(unless (find-package "TRAVERSABILITY_MSGS::CHECKFOOTPRINTPATHRESPONSE")
  (make-package "TRAVERSABILITY_MSGS::CHECKFOOTPRINTPATHRESPONSE"))

(in-package "ROS")





(defclass traversability_msgs::CheckFootprintPathRequest
  :super ros::object
  :slots (_path ))

(defmethod traversability_msgs::CheckFootprintPathRequest
  (:init
   (&key
    ((:path __path) ())
    )
   (send-super :init)
   (setq _path __path)
   self)
  (:path
   (&rest __path)
   (if (keywordp (car __path))
       (send* _path __path)
     (progn
       (if __path (setq _path (car __path)))
       _path)))
  (:serialization-length
   ()
   (+
    ;; traversability_msgs/FootprintPath[] _path
    (apply #'+ (send-all _path :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; traversability_msgs/FootprintPath[] _path
     (write-long (length _path) s)
     (dolist (elem _path)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; traversability_msgs/FootprintPath[] _path
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _path (let (r) (dotimes (i n) (push (instance traversability_msgs::FootprintPath :init) r)) r))
     (dolist (elem- _path)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(defclass traversability_msgs::CheckFootprintPathResponse
  :super ros::object
  :slots (_result ))

(defmethod traversability_msgs::CheckFootprintPathResponse
  (:init
   (&key
    ((:result __result) ())
    )
   (send-super :init)
   (setq _result __result)
   self)
  (:result
   (&rest __result)
   (if (keywordp (car __result))
       (send* _result __result)
     (progn
       (if __result (setq _result (car __result)))
       _result)))
  (:serialization-length
   ()
   (+
    ;; traversability_msgs/TraversabilityResult[] _result
    (apply #'+ (send-all _result :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; traversability_msgs/TraversabilityResult[] _result
     (write-long (length _result) s)
     (dolist (elem _result)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; traversability_msgs/TraversabilityResult[] _result
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _result (let (r) (dotimes (i n) (push (instance traversability_msgs::TraversabilityResult :init) r)) r))
     (dolist (elem- _result)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(defclass traversability_msgs::CheckFootprintPath
  :super ros::object
  :slots ())

(setf (get traversability_msgs::CheckFootprintPath :md5sum-) "c63bb28cc303394e26806233caf914bd")
(setf (get traversability_msgs::CheckFootprintPath :datatype-) "traversability_msgs/CheckFootprintPath")
(setf (get traversability_msgs::CheckFootprintPath :request) traversability_msgs::CheckFootprintPathRequest)
(setf (get traversability_msgs::CheckFootprintPath :response) traversability_msgs::CheckFootprintPathResponse)

(defmethod traversability_msgs::CheckFootprintPathRequest
  (:response () (instance traversability_msgs::CheckFootprintPathResponse :init)))

(setf (get traversability_msgs::CheckFootprintPathRequest :md5sum-) "c63bb28cc303394e26806233caf914bd")
(setf (get traversability_msgs::CheckFootprintPathRequest :datatype-) "traversability_msgs/CheckFootprintPathRequest")
(setf (get traversability_msgs::CheckFootprintPathRequest :definition-)
      "# Footprint path to check.
traversability_msgs/FootprintPath[] path


================================================================================
MSG: traversability_msgs/FootprintPath
# Path of the footprint defined as array of connected poses.
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
---

# Traversability results
traversability_msgs/TraversabilityResult[] result

================================================================================
MSG: traversability_msgs/TraversabilityResult
# If path is safe to traverse.
bool is_safe

# Estimate of the traversability of the path.
# Ranges from 0 to 1 where 0 means not traversable and 1 highly traversable.
float64 traversability

# Area of the footprint path.
")

(setf (get traversability_msgs::CheckFootprintPathResponse :md5sum-) "c63bb28cc303394e26806233caf914bd")
(setf (get traversability_msgs::CheckFootprintPathResponse :datatype-) "traversability_msgs/CheckFootprintPathResponse")
(setf (get traversability_msgs::CheckFootprintPathResponse :definition-)
      "# Footprint path to check.
traversability_msgs/FootprintPath[] path


================================================================================
MSG: traversability_msgs/FootprintPath
# Path of the footprint defined as array of connected poses.
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
---

# Traversability results
traversability_msgs/TraversabilityResult[] result

================================================================================
MSG: traversability_msgs/TraversabilityResult
# If path is safe to traverse.
bool is_safe

# Estimate of the traversability of the path.
# Ranges from 0 to 1 where 0 means not traversable and 1 highly traversable.
float64 traversability

# Area of the footprint path.
")



(provide :traversability_msgs/CheckFootprintPath "c63bb28cc303394e26806233caf914bd")


