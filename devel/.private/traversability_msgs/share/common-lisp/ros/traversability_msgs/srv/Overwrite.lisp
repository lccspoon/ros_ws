; Auto-generated. Do not edit!


(cl:in-package traversability_msgs-srv)


;//! \htmlinclude Overwrite-request.msg.html

(cl:defclass <Overwrite-request> (roslisp-msg-protocol:ros-message)
  ((enable
    :reader enable
    :initarg :enable
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Overwrite-request (<Overwrite-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Overwrite-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Overwrite-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name traversability_msgs-srv:<Overwrite-request> is deprecated: use traversability_msgs-srv:Overwrite-request instead.")))

(cl:ensure-generic-function 'enable-val :lambda-list '(m))
(cl:defmethod enable-val ((m <Overwrite-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traversability_msgs-srv:enable-val is deprecated.  Use traversability_msgs-srv:enable instead.")
  (enable m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Overwrite-request>) ostream)
  "Serializes a message object of type '<Overwrite-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'enable) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Overwrite-request>) istream)
  "Deserializes a message object of type '<Overwrite-request>"
    (cl:setf (cl:slot-value msg 'enable) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Overwrite-request>)))
  "Returns string type for a service object of type '<Overwrite-request>"
  "traversability_msgs/OverwriteRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Overwrite-request)))
  "Returns string type for a service object of type 'Overwrite-request"
  "traversability_msgs/OverwriteRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Overwrite-request>)))
  "Returns md5sum for a message object of type '<Overwrite-request>"
  "8c1211af706069c994c06e00eb59ac9e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Overwrite-request)))
  "Returns md5sum for a message object of type 'Overwrite-request"
  "8c1211af706069c994c06e00eb59ac9e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Overwrite-request>)))
  "Returns full string definition for message of type '<Overwrite-request>"
  (cl:format cl:nil "# True to enable, false for disable.~%bool enable~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Overwrite-request)))
  "Returns full string definition for message of type 'Overwrite-request"
  (cl:format cl:nil "# True to enable, false for disable.~%bool enable~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Overwrite-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Overwrite-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Overwrite-request
    (cl:cons ':enable (enable msg))
))
;//! \htmlinclude Overwrite-response.msg.html

(cl:defclass <Overwrite-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Overwrite-response (<Overwrite-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Overwrite-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Overwrite-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name traversability_msgs-srv:<Overwrite-response> is deprecated: use traversability_msgs-srv:Overwrite-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Overwrite-response>) ostream)
  "Serializes a message object of type '<Overwrite-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Overwrite-response>) istream)
  "Deserializes a message object of type '<Overwrite-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Overwrite-response>)))
  "Returns string type for a service object of type '<Overwrite-response>"
  "traversability_msgs/OverwriteResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Overwrite-response)))
  "Returns string type for a service object of type 'Overwrite-response"
  "traversability_msgs/OverwriteResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Overwrite-response>)))
  "Returns md5sum for a message object of type '<Overwrite-response>"
  "8c1211af706069c994c06e00eb59ac9e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Overwrite-response)))
  "Returns md5sum for a message object of type 'Overwrite-response"
  "8c1211af706069c994c06e00eb59ac9e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Overwrite-response>)))
  "Returns full string definition for message of type '<Overwrite-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Overwrite-response)))
  "Returns full string definition for message of type 'Overwrite-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Overwrite-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Overwrite-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Overwrite-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Overwrite)))
  'Overwrite-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Overwrite)))
  'Overwrite-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Overwrite)))
  "Returns string type for a service object of type '<Overwrite>"
  "traversability_msgs/Overwrite")