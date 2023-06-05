; Auto-generated. Do not edit!


(cl:in-package traversability_msgs-srv)


;//! \htmlinclude CheckFootprintPath-request.msg.html

(cl:defclass <CheckFootprintPath-request> (roslisp-msg-protocol:ros-message)
  ((path
    :reader path
    :initarg :path
    :type (cl:vector traversability_msgs-msg:FootprintPath)
   :initform (cl:make-array 0 :element-type 'traversability_msgs-msg:FootprintPath :initial-element (cl:make-instance 'traversability_msgs-msg:FootprintPath))))
)

(cl:defclass CheckFootprintPath-request (<CheckFootprintPath-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CheckFootprintPath-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CheckFootprintPath-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name traversability_msgs-srv:<CheckFootprintPath-request> is deprecated: use traversability_msgs-srv:CheckFootprintPath-request instead.")))

(cl:ensure-generic-function 'path-val :lambda-list '(m))
(cl:defmethod path-val ((m <CheckFootprintPath-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traversability_msgs-srv:path-val is deprecated.  Use traversability_msgs-srv:path instead.")
  (path m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CheckFootprintPath-request>) ostream)
  "Serializes a message object of type '<CheckFootprintPath-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'path))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'path))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CheckFootprintPath-request>) istream)
  "Deserializes a message object of type '<CheckFootprintPath-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'path) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'path)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'traversability_msgs-msg:FootprintPath))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CheckFootprintPath-request>)))
  "Returns string type for a service object of type '<CheckFootprintPath-request>"
  "traversability_msgs/CheckFootprintPathRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CheckFootprintPath-request)))
  "Returns string type for a service object of type 'CheckFootprintPath-request"
  "traversability_msgs/CheckFootprintPathRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CheckFootprintPath-request>)))
  "Returns md5sum for a message object of type '<CheckFootprintPath-request>"
  "c63bb28cc303394e26806233caf914bd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CheckFootprintPath-request)))
  "Returns md5sum for a message object of type 'CheckFootprintPath-request"
  "c63bb28cc303394e26806233caf914bd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CheckFootprintPath-request>)))
  "Returns full string definition for message of type '<CheckFootprintPath-request>"
  (cl:format cl:nil "# Footprint path to check.~%traversability_msgs/FootprintPath[] path~%~%~%================================================================================~%MSG: traversability_msgs/FootprintPath~%# Path of the footprint defined as array of connected poses.~%# Poses are connected piece-wise linear in the order of the array.~%geometry_msgs/PoseArray poses~%~%# Either: Define footprint radius.~%float64 radius ~%~%# Or: Define footprint as polygon in the robot base frame.~%# Polygon is used if it is defined, otherwise radius is used.~%geometry_msgs/PolygonStamped footprint~%~%# Use conservative footprint. Only available if polygon is used.~%bool conservative~%~%# Compute untraversable polygon in the checked area for traversability. If true, computation demand is higher.~%bool compute_untraversable_polygon~%~%================================================================================~%MSG: geometry_msgs/PoseArray~%# An array of poses with a header for global reference.~%~%Header header~%~%Pose[] poses~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/PolygonStamped~%# This represents a Polygon with reference coordinate frame and timestamp~%Header header~%Polygon polygon~%~%================================================================================~%MSG: geometry_msgs/Polygon~%#A specification of a polygon where the first and last points are assumed to be connected~%Point32[] points~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CheckFootprintPath-request)))
  "Returns full string definition for message of type 'CheckFootprintPath-request"
  (cl:format cl:nil "# Footprint path to check.~%traversability_msgs/FootprintPath[] path~%~%~%================================================================================~%MSG: traversability_msgs/FootprintPath~%# Path of the footprint defined as array of connected poses.~%# Poses are connected piece-wise linear in the order of the array.~%geometry_msgs/PoseArray poses~%~%# Either: Define footprint radius.~%float64 radius ~%~%# Or: Define footprint as polygon in the robot base frame.~%# Polygon is used if it is defined, otherwise radius is used.~%geometry_msgs/PolygonStamped footprint~%~%# Use conservative footprint. Only available if polygon is used.~%bool conservative~%~%# Compute untraversable polygon in the checked area for traversability. If true, computation demand is higher.~%bool compute_untraversable_polygon~%~%================================================================================~%MSG: geometry_msgs/PoseArray~%# An array of poses with a header for global reference.~%~%Header header~%~%Pose[] poses~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/PolygonStamped~%# This represents a Polygon with reference coordinate frame and timestamp~%Header header~%Polygon polygon~%~%================================================================================~%MSG: geometry_msgs/Polygon~%#A specification of a polygon where the first and last points are assumed to be connected~%Point32[] points~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CheckFootprintPath-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'path) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CheckFootprintPath-request>))
  "Converts a ROS message object to a list"
  (cl:list 'CheckFootprintPath-request
    (cl:cons ':path (path msg))
))
;//! \htmlinclude CheckFootprintPath-response.msg.html

(cl:defclass <CheckFootprintPath-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type (cl:vector traversability_msgs-msg:TraversabilityResult)
   :initform (cl:make-array 0 :element-type 'traversability_msgs-msg:TraversabilityResult :initial-element (cl:make-instance 'traversability_msgs-msg:TraversabilityResult))))
)

(cl:defclass CheckFootprintPath-response (<CheckFootprintPath-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CheckFootprintPath-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CheckFootprintPath-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name traversability_msgs-srv:<CheckFootprintPath-response> is deprecated: use traversability_msgs-srv:CheckFootprintPath-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <CheckFootprintPath-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traversability_msgs-srv:result-val is deprecated.  Use traversability_msgs-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CheckFootprintPath-response>) ostream)
  "Serializes a message object of type '<CheckFootprintPath-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'result))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'result))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CheckFootprintPath-response>) istream)
  "Deserializes a message object of type '<CheckFootprintPath-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'result) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'result)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'traversability_msgs-msg:TraversabilityResult))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CheckFootprintPath-response>)))
  "Returns string type for a service object of type '<CheckFootprintPath-response>"
  "traversability_msgs/CheckFootprintPathResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CheckFootprintPath-response)))
  "Returns string type for a service object of type 'CheckFootprintPath-response"
  "traversability_msgs/CheckFootprintPathResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CheckFootprintPath-response>)))
  "Returns md5sum for a message object of type '<CheckFootprintPath-response>"
  "c63bb28cc303394e26806233caf914bd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CheckFootprintPath-response)))
  "Returns md5sum for a message object of type 'CheckFootprintPath-response"
  "c63bb28cc303394e26806233caf914bd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CheckFootprintPath-response>)))
  "Returns full string definition for message of type '<CheckFootprintPath-response>"
  (cl:format cl:nil "~%# Traversability results~%traversability_msgs/TraversabilityResult[] result~%~%================================================================================~%MSG: traversability_msgs/TraversabilityResult~%# If path is safe to traverse.~%bool is_safe~%~%# Estimate of the traversability of the path.~%# Ranges from 0 to 1 where 0 means not traversable and 1 highly traversable.~%float64 traversability~%~%# Area of the footprint path.~%float64 area~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CheckFootprintPath-response)))
  "Returns full string definition for message of type 'CheckFootprintPath-response"
  (cl:format cl:nil "~%# Traversability results~%traversability_msgs/TraversabilityResult[] result~%~%================================================================================~%MSG: traversability_msgs/TraversabilityResult~%# If path is safe to traverse.~%bool is_safe~%~%# Estimate of the traversability of the path.~%# Ranges from 0 to 1 where 0 means not traversable and 1 highly traversable.~%float64 traversability~%~%# Area of the footprint path.~%float64 area~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CheckFootprintPath-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'result) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CheckFootprintPath-response>))
  "Converts a ROS message object to a list"
  (cl:list 'CheckFootprintPath-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'CheckFootprintPath)))
  'CheckFootprintPath-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'CheckFootprintPath)))
  'CheckFootprintPath-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CheckFootprintPath)))
  "Returns string type for a service object of type '<CheckFootprintPath>"
  "traversability_msgs/CheckFootprintPath")