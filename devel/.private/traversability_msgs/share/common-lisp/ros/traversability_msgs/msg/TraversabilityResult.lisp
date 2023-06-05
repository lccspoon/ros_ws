; Auto-generated. Do not edit!


(cl:in-package traversability_msgs-msg)


;//! \htmlinclude TraversabilityResult.msg.html

(cl:defclass <TraversabilityResult> (roslisp-msg-protocol:ros-message)
  ((is_safe
    :reader is_safe
    :initarg :is_safe
    :type cl:boolean
    :initform cl:nil)
   (traversability
    :reader traversability
    :initarg :traversability
    :type cl:float
    :initform 0.0)
   (area
    :reader area
    :initarg :area
    :type cl:float
    :initform 0.0))
)

(cl:defclass TraversabilityResult (<TraversabilityResult>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TraversabilityResult>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TraversabilityResult)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name traversability_msgs-msg:<TraversabilityResult> is deprecated: use traversability_msgs-msg:TraversabilityResult instead.")))

(cl:ensure-generic-function 'is_safe-val :lambda-list '(m))
(cl:defmethod is_safe-val ((m <TraversabilityResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traversability_msgs-msg:is_safe-val is deprecated.  Use traversability_msgs-msg:is_safe instead.")
  (is_safe m))

(cl:ensure-generic-function 'traversability-val :lambda-list '(m))
(cl:defmethod traversability-val ((m <TraversabilityResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traversability_msgs-msg:traversability-val is deprecated.  Use traversability_msgs-msg:traversability instead.")
  (traversability m))

(cl:ensure-generic-function 'area-val :lambda-list '(m))
(cl:defmethod area-val ((m <TraversabilityResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traversability_msgs-msg:area-val is deprecated.  Use traversability_msgs-msg:area instead.")
  (area m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TraversabilityResult>) ostream)
  "Serializes a message object of type '<TraversabilityResult>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'is_safe) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'traversability))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'area))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TraversabilityResult>) istream)
  "Deserializes a message object of type '<TraversabilityResult>"
    (cl:setf (cl:slot-value msg 'is_safe) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'traversability) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'area) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TraversabilityResult>)))
  "Returns string type for a message object of type '<TraversabilityResult>"
  "traversability_msgs/TraversabilityResult")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TraversabilityResult)))
  "Returns string type for a message object of type 'TraversabilityResult"
  "traversability_msgs/TraversabilityResult")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TraversabilityResult>)))
  "Returns md5sum for a message object of type '<TraversabilityResult>"
  "14ffe3323c91cd823bef7a313714954e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TraversabilityResult)))
  "Returns md5sum for a message object of type 'TraversabilityResult"
  "14ffe3323c91cd823bef7a313714954e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TraversabilityResult>)))
  "Returns full string definition for message of type '<TraversabilityResult>"
  (cl:format cl:nil "# If path is safe to traverse.~%bool is_safe~%~%# Estimate of the traversability of the path.~%# Ranges from 0 to 1 where 0 means not traversable and 1 highly traversable.~%float64 traversability~%~%# Area of the footprint path.~%float64 area~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TraversabilityResult)))
  "Returns full string definition for message of type 'TraversabilityResult"
  (cl:format cl:nil "# If path is safe to traverse.~%bool is_safe~%~%# Estimate of the traversability of the path.~%# Ranges from 0 to 1 where 0 means not traversable and 1 highly traversable.~%float64 traversability~%~%# Area of the footprint path.~%float64 area~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TraversabilityResult>))
  (cl:+ 0
     1
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TraversabilityResult>))
  "Converts a ROS message object to a list"
  (cl:list 'TraversabilityResult
    (cl:cons ':is_safe (is_safe msg))
    (cl:cons ':traversability (traversability msg))
    (cl:cons ':area (area msg))
))
