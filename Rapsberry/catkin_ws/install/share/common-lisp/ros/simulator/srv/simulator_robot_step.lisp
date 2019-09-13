; Auto-generated. Do not edit!


(cl:in-package simulator-srv)


;//! \htmlinclude simulator_robot_step-request.msg.html

(cl:defclass <simulator_robot_step-request> (roslisp-msg-protocol:ros-message)
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

(cl:defclass simulator_robot_step-request (<simulator_robot_step-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <simulator_robot_step-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'simulator_robot_step-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name simulator-srv:<simulator_robot_step-request> is deprecated: use simulator-srv:simulator_robot_step-request instead.")))

(cl:ensure-generic-function 'theta-val :lambda-list '(m))
(cl:defmethod theta-val ((m <simulator_robot_step-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simulator-srv:theta-val is deprecated.  Use simulator-srv:theta instead.")
  (theta m))

(cl:ensure-generic-function 'distance-val :lambda-list '(m))
(cl:defmethod distance-val ((m <simulator_robot_step-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simulator-srv:distance-val is deprecated.  Use simulator-srv:distance instead.")
  (distance m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <simulator_robot_step-request>) ostream)
  "Serializes a message object of type '<simulator_robot_step-request>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <simulator_robot_step-request>) istream)
  "Deserializes a message object of type '<simulator_robot_step-request>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<simulator_robot_step-request>)))
  "Returns string type for a service object of type '<simulator_robot_step-request>"
  "simulator/simulator_robot_stepRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'simulator_robot_step-request)))
  "Returns string type for a service object of type 'simulator_robot_step-request"
  "simulator/simulator_robot_stepRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<simulator_robot_step-request>)))
  "Returns md5sum for a message object of type '<simulator_robot_step-request>"
  "851b8b2ccdd280507cfab4425c086df0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'simulator_robot_step-request)))
  "Returns md5sum for a message object of type 'simulator_robot_step-request"
  "851b8b2ccdd280507cfab4425c086df0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<simulator_robot_step-request>)))
  "Returns full string definition for message of type '<simulator_robot_step-request>"
  (cl:format cl:nil "float32 theta~%float32 distance~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'simulator_robot_step-request)))
  "Returns full string definition for message of type 'simulator_robot_step-request"
  (cl:format cl:nil "float32 theta~%float32 distance~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <simulator_robot_step-request>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <simulator_robot_step-request>))
  "Converts a ROS message object to a list"
  (cl:list 'simulator_robot_step-request
    (cl:cons ':theta (theta msg))
    (cl:cons ':distance (distance msg))
))
;//! \htmlinclude simulator_robot_step-response.msg.html

(cl:defclass <simulator_robot_step-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:integer
    :initform 0)
   (robot_x
    :reader robot_x
    :initarg :robot_x
    :type cl:float
    :initform 0.0)
   (robot_y
    :reader robot_y
    :initarg :robot_y
    :type cl:float
    :initform 0.0)
   (theta
    :reader theta
    :initarg :theta
    :type cl:float
    :initform 0.0))
)

(cl:defclass simulator_robot_step-response (<simulator_robot_step-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <simulator_robot_step-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'simulator_robot_step-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name simulator-srv:<simulator_robot_step-response> is deprecated: use simulator-srv:simulator_robot_step-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <simulator_robot_step-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simulator-srv:success-val is deprecated.  Use simulator-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'robot_x-val :lambda-list '(m))
(cl:defmethod robot_x-val ((m <simulator_robot_step-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simulator-srv:robot_x-val is deprecated.  Use simulator-srv:robot_x instead.")
  (robot_x m))

(cl:ensure-generic-function 'robot_y-val :lambda-list '(m))
(cl:defmethod robot_y-val ((m <simulator_robot_step-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simulator-srv:robot_y-val is deprecated.  Use simulator-srv:robot_y instead.")
  (robot_y m))

(cl:ensure-generic-function 'theta-val :lambda-list '(m))
(cl:defmethod theta-val ((m <simulator_robot_step-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simulator-srv:theta-val is deprecated.  Use simulator-srv:theta instead.")
  (theta m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <simulator_robot_step-response>) ostream)
  "Serializes a message object of type '<simulator_robot_step-response>"
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
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'robot_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'robot_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'theta))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <simulator_robot_step-response>) istream)
  "Deserializes a message object of type '<simulator_robot_step-response>"
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
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'robot_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'robot_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'theta) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<simulator_robot_step-response>)))
  "Returns string type for a service object of type '<simulator_robot_step-response>"
  "simulator/simulator_robot_stepResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'simulator_robot_step-response)))
  "Returns string type for a service object of type 'simulator_robot_step-response"
  "simulator/simulator_robot_stepResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<simulator_robot_step-response>)))
  "Returns md5sum for a message object of type '<simulator_robot_step-response>"
  "851b8b2ccdd280507cfab4425c086df0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'simulator_robot_step-response)))
  "Returns md5sum for a message object of type 'simulator_robot_step-response"
  "851b8b2ccdd280507cfab4425c086df0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<simulator_robot_step-response>)))
  "Returns full string definition for message of type '<simulator_robot_step-response>"
  (cl:format cl:nil "int64 success~%float32 robot_x~%float32 robot_y~%float32 theta~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'simulator_robot_step-response)))
  "Returns full string definition for message of type 'simulator_robot_step-response"
  (cl:format cl:nil "int64 success~%float32 robot_x~%float32 robot_y~%float32 theta~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <simulator_robot_step-response>))
  (cl:+ 0
     8
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <simulator_robot_step-response>))
  "Converts a ROS message object to a list"
  (cl:list 'simulator_robot_step-response
    (cl:cons ':success (success msg))
    (cl:cons ':robot_x (robot_x msg))
    (cl:cons ':robot_y (robot_y msg))
    (cl:cons ':theta (theta msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'simulator_robot_step)))
  'simulator_robot_step-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'simulator_robot_step)))
  'simulator_robot_step-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'simulator_robot_step)))
  "Returns string type for a service object of type '<simulator_robot_step>"
  "simulator/simulator_robot_step")