; Auto-generated. Do not edit!


(cl:in-package simulator-srv)


;//! \htmlinclude robot_step-request.msg.html

(cl:defclass <robot_step-request> (roslisp-msg-protocol:ros-message)
  ((theta
    :reader theta
    :initarg :theta
    :type cl:float
    :initform 0.0)
   (distance
    :reader distance
    :initarg :distance
    :type cl:float
    :initform 0.0))
)

(cl:defclass robot_step-request (<robot_step-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <robot_step-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'robot_step-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name simulator-srv:<robot_step-request> is deprecated: use simulator-srv:robot_step-request instead.")))

(cl:ensure-generic-function 'theta-val :lambda-list '(m))
(cl:defmethod theta-val ((m <robot_step-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simulator-srv:theta-val is deprecated.  Use simulator-srv:theta instead.")
  (theta m))

(cl:ensure-generic-function 'distance-val :lambda-list '(m))
(cl:defmethod distance-val ((m <robot_step-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simulator-srv:distance-val is deprecated.  Use simulator-srv:distance instead.")
  (distance m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <robot_step-request>) ostream)
  "Serializes a message object of type '<robot_step-request>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'theta))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'distance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <robot_step-request>) istream)
  "Deserializes a message object of type '<robot_step-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'theta) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'distance) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<robot_step-request>)))
  "Returns string type for a service object of type '<robot_step-request>"
  "simulator/robot_stepRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'robot_step-request)))
  "Returns string type for a service object of type 'robot_step-request"
  "simulator/robot_stepRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<robot_step-request>)))
  "Returns md5sum for a message object of type '<robot_step-request>"
  "aa748166a5bb70fc4aeb3cae4da517f9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'robot_step-request)))
  "Returns md5sum for a message object of type 'robot_step-request"
  "aa748166a5bb70fc4aeb3cae4da517f9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<robot_step-request>)))
  "Returns full string definition for message of type '<robot_step-request>"
  (cl:format cl:nil "float32 theta~%float32 distance~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'robot_step-request)))
  "Returns full string definition for message of type 'robot_step-request"
  (cl:format cl:nil "float32 theta~%float32 distance~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <robot_step-request>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <robot_step-request>))
  "Converts a ROS message object to a list"
  (cl:list 'robot_step-request
    (cl:cons ':theta (theta msg))
    (cl:cons ':distance (distance msg))
))
;//! \htmlinclude robot_step-response.msg.html

(cl:defclass <robot_step-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:integer
    :initform 0))
)

(cl:defclass robot_step-response (<robot_step-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <robot_step-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'robot_step-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name simulator-srv:<robot_step-response> is deprecated: use simulator-srv:robot_step-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <robot_step-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simulator-srv:success-val is deprecated.  Use simulator-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <robot_step-response>) ostream)
  "Serializes a message object of type '<robot_step-response>"
  (cl:let* ((signed (cl:slot-value msg 'success)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <robot_step-response>) istream)
  "Deserializes a message object of type '<robot_step-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'success) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<robot_step-response>)))
  "Returns string type for a service object of type '<robot_step-response>"
  "simulator/robot_stepResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'robot_step-response)))
  "Returns string type for a service object of type 'robot_step-response"
  "simulator/robot_stepResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<robot_step-response>)))
  "Returns md5sum for a message object of type '<robot_step-response>"
  "aa748166a5bb70fc4aeb3cae4da517f9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'robot_step-response)))
  "Returns md5sum for a message object of type 'robot_step-response"
  "aa748166a5bb70fc4aeb3cae4da517f9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<robot_step-response>)))
  "Returns full string definition for message of type '<robot_step-response>"
  (cl:format cl:nil "int64 success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'robot_step-response)))
  "Returns full string definition for message of type 'robot_step-response"
  (cl:format cl:nil "int64 success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <robot_step-response>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <robot_step-response>))
  "Converts a ROS message object to a list"
  (cl:list 'robot_step-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'robot_step)))
  'robot_step-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'robot_step)))
  'robot_step-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'robot_step)))
  "Returns string type for a service object of type '<robot_step>"
  "simulator/robot_step")