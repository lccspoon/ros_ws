; Auto-generated. Do not edit!


(cl:in-package traversability_msgs-msg)


;//! \htmlinclude FootprintPath.msg.html

(cl:defclass <FootprintPath> (roslisp-msg-protocol:ros-message)
  ((poses
    :reader poses
    :initarg :poses
    :type geometry_msgs-msg:PoseArray
    :initform (cl:make-instance 'geometry_msgs-msg:PoseArray))
   (radius
    :reader radius
    :initarg :radius
    :type cl:float
    :initform 0.0)
   (footprint
    :reader footprint
    :initarg :footprint
    :type geometry_msgs-msg:PolygonStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PolygonStamped))
   (conservative
    :reader conservative
    :initarg :conservative
    :type cl:boolean
    :initform cl:nil)
   (compute_untraversable_polygon
    :reader compute_untraversable_polygon
    :initarg :compute_untraversable_polygon
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass FootprintPath (<FootprintPath>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FootprintPath>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FootprintPath)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name traversability_msgs-msg:<FootprintPath> is deprecated: use traversability_msgs-msg:FootprintPath instead.")))

(cl:ensure-generic-function 'poses-val :lambda-list '(m))
(cl:defmethod poses-val ((m <FootprintPath>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traversability_msgs-msg:poses-val is deprecated.  Use traversability_msgs-msg:poses instead.")
  (poses m))

(cl:ensure-generic-function 'radius-val :lambda-list '(m))
(cl:defmethod radius-val ((m <FootprintPath>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traversability_msgs-msg:radius-val is deprecated.  Use traversability_msgs-msg:radius instead.")
  (radius m))

(cl:ensure-generic-function 'footprint-val :lambda-list '(m))
(cl:defmethod footprint-val ((m <FootprintPath>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traversability_msgs-msg:footprint-val is deprecated.  Use traversability_msgs-msg:footprint instead.")
  (footprint m))

(cl:ensure-generic-function 'conservative-val :lambda-list '(m))
(cl:defmethod conservative-val ((m <FootprintPath>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traversability_msgs-msg:conservative-val is deprecated.  Use traversability_msgs-msg:conservative instead.")
  (conservative m))

(cl:ensure-generic-function 'compute_untraversable_polygon-val :lambda-list '(m))
(cl:defmethod compute_untraversable_polygon-val ((m <FootprintPath>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traversability_msgs-msg:compute_untraversable_polygon-val is deprecated.  Use traversability_msgs-msg:compute_untraversable_polygon instead.")
  (compute_untraversable_polygon m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FootprintPath>) ostream)
  "Serializes a message object of type '<FootprintPath>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'poses) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'radius))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'footprint) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'conservative) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'compute_untraversable_polygon) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FootprintPath>) istream)
  "Deserializes a message object of type '<FootprintPath>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'poses) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'radius) (roslisp-utils:decode-double-float-bits bits)))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'footprint) istream)
    (cl:setf (cl:slot-value msg 'conservative) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'compute_untraversable_polygon) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FootprintPath>)))
  "Returns string type for a message object of type '<FootprintPath>"
  "traversability_msgs/FootprintPath")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FootprintPath)))
  "Returns string type for a message object of type 'FootprintPath"
  "traversability_msgs/FootprintPath")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FootprintPath>)))
  "Returns md5sum for a message object of type '<FootprintPath>"
  "6a088c23f98a81d850097dff6ef7145d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FootprintPath)))
  "Returns md5sum for a message object of type 'FootprintPath"
  "6a088c23f98a81d850097dff6ef7145d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FootprintPath>)))
  "Returns full string definition for message of type '<FootprintPath>"
  (cl:format cl:nil "# Path of the footprint defined as array of connected poses.~%# Poses are connected piece-wise linear in the order of the array.~%geometry_msgs/PoseArray poses~%~%# Either: Define footprint radius.~%float64 radius ~%~%# Or: Define footprint as polygon in the robot base frame.~%# Polygon is used if it is defined, otherwise radius is used.~%geometry_msgs/PolygonStamped footprint~%~%# Use conservative footprint. Only available if polygon is used.~%bool conservative~%~%# Compute untraversable polygon in the checked area for traversability. If true, computation demand is higher.~%bool compute_untraversable_polygon~%~%================================================================================~%MSG: geometry_msgs/PoseArray~%# An array of poses with a header for global reference.~%~%Header header~%~%Pose[] poses~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/PolygonStamped~%# This represents a Polygon with reference coordinate frame and timestamp~%Header header~%Polygon polygon~%~%================================================================================~%MSG: geometry_msgs/Polygon~%#A specification of a polygon where the first and last points are assumed to be connected~%Point32[] points~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FootprintPath)))
  "Returns full string definition for message of type 'FootprintPath"
  (cl:format cl:nil "# Path of the footprint defined as array of connected poses.~%# Poses are connected piece-wise linear in the order of the array.~%geometry_msgs/PoseArray poses~%~%# Either: Define footprint radius.~%float64 radius ~%~%# Or: Define footprint as polygon in the robot base frame.~%# Polygon is used if it is defined, otherwise radius is used.~%geometry_msgs/PolygonStamped footprint~%~%# Use conservative footprint. Only available if polygon is used.~%bool conservative~%~%# Compute untraversable polygon in the checked area for traversability. If true, computation demand is higher.~%bool compute_untraversable_polygon~%~%================================================================================~%MSG: geometry_msgs/PoseArray~%# An array of poses with a header for global reference.~%~%Header header~%~%Pose[] poses~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/PolygonStamped~%# This represents a Polygon with reference coordinate frame and timestamp~%Header header~%Polygon polygon~%~%================================================================================~%MSG: geometry_msgs/Polygon~%#A specification of a polygon where the first and last points are assumed to be connected~%Point32[] points~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FootprintPath>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'poses))
     8
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'footprint))
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FootprintPath>))
  "Converts a ROS message object to a list"
  (cl:list 'FootprintPath
    (cl:cons ':poses (poses msg))
    (cl:cons ':radius (radius msg))
    (cl:cons ':footprint (footprint msg))
    (cl:cons ':conservative (conservative msg))
    (cl:cons ':compute_untraversable_polygon (compute_untraversable_polygon msg))
))
