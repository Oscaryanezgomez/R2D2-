; Auto-generated. Do not edit!


(cl:in-package simulator-srv)


;//! \htmlinclude simulator_robot_laser_values-request.msg.html

(cl:defclass <simulator_robot_laser_values-request> (roslisp-msg-protocol:ros-message)
  ((sensors
    :reader sensors
    :initarg :sensors
    :type (cl:vector cl:float)
   :initform (cl:make-array 100 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass simulator_robot_laser_values-request (<simulator_robot_laser_values-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <simulator_robot_laser_values-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'simulator_robot_laser_values-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name simulator-srv:<simulator_robot_laser_values-request> is deprecated: use simulator-srv:simulator_robot_laser_values-request instead.")))

(cl:ensure-generic-function 'sensors-val :lambda-list '(m))
(cl:defmethod sensors-val ((m <simulator_robot_laser_values-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simulator-srv:sensors-val is deprecated.  Use simulator-srv:sensors instead.")
  (sensors m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <simulator_robot_laser_values-request>) ostream)
  "Serializes a message object of type '<simulator_robot_laser_values-request>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'sensors))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <simulator_robot_laser_values-request>) istream)
  "Deserializes a message object of type '<simulator_robot_laser_values-request>"
  (cl:setf (cl:slot-value msg 'sensors) (cl:make-array 100))
  (cl:let ((vals (cl:slot-value msg 'sensors)))
    (cl:dotimes (i 100)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<simulator_robot_laser_values-request>)))
  "Returns string type for a service object of type '<simulator_robot_laser_values-request>"
  "simulator/simulator_robot_laser_valuesRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'simulator_robot_laser_values-request)))
  "Returns string type for a service object of type 'simulator_robot_laser_values-request"
  "simulator/simulator_robot_laser_valuesRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<simulator_robot_laser_values-request>)))
  "Returns md5sum for a message object of type '<simulator_robot_laser_values-request>"
  "fed2e1160fc2ee00e074cbac3dfb8139")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'simulator_robot_laser_values-request)))
  "Returns md5sum for a message object of type 'simulator_robot_laser_values-request"
  "fed2e1160fc2ee00e074cbac3dfb8139")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<simulator_robot_laser_values-request>)))
  "Returns full string definition for message of type '<simulator_robot_laser_values-request>"
  (cl:format cl:nil "float32[100] sensors~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'simulator_robot_laser_values-request)))
  "Returns full string definition for message of type 'simulator_robot_laser_values-request"
  (cl:format cl:nil "float32[100] sensors~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <simulator_robot_laser_values-request>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'sensors) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <simulator_robot_laser_values-request>))
  "Converts a ROS message object to a list"
  (cl:list 'simulator_robot_laser_values-request
    (cl:cons ':sensors (sensors msg))
))
;//! \htmlinclude simulator_robot_laser_values-response.msg.html

(cl:defclass <simulator_robot_laser_values-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:integer
    :initform 0))
)

(cl:defclass simulator_robot_laser_values-response (<simulator_robot_laser_values-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <simulator_robot_laser_values-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'simulator_robot_laser_values-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name simulator-srv:<simulator_robot_laser_values-response> is deprecated: use simulator-srv:simulator_robot_laser_values-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <simulator_robot_laser_values-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simulator-srv:success-val is deprecated.  Use simulator-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <simulator_robot_laser_values-response>) ostream)
  "Serializes a message object of type '<simulator_robot_laser_values-response>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <simulator_robot_laser_values-response>) istream)
  "Deserializes a message object of type '<simulator_robot_laser_values-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<simulator_robot_laser_values-response>)))
  "Returns string type for a service object of type '<simulator_robot_laser_values-response>"
  "simulator/simulator_robot_laser_valuesResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'simulator_robot_laser_values-response)))
  "Returns string type for a service object of type 'simulator_robot_laser_values-response"
  "simulator/simulator_robot_laser_valuesResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<simulator_robot_laser_values-response>)))
  "Returns md5sum for a message object of type '<simulator_robot_laser_values-response>"
  "fed2e1160fc2ee00e074cbac3dfb8139")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'simulator_robot_laser_values-response)))
  "Returns md5sum for a message object of type 'simulator_robot_laser_values-response"
  "fed2e1160fc2ee00e074cbac3dfb8139")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<simulator_robot_laser_values-response>)))
  "Returns full string definition for message of type '<simulator_robot_laser_values-response>"
  (cl:format cl:nil "int64 success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'simulator_robot_laser_values-response)))
  "Returns full string definition for message of type 'simulator_robot_laser_values-response"
  (cl:format cl:nil "int64 success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <simulator_robot_laser_values-response>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <simulator_robot_laser_values-response>))
  "Converts a ROS message object to a list"
  (cl:list 'simulator_robot_laser_values-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'simulator_robot_laser_values)))
  'simulator_robot_laser_values-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'simulator_robot_laser_values)))
  'simulator_robot_laser_values-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'simulator_robot_laser_values)))
  "Returns string type for a service object of type '<simulator_robot_laser_values>"
  "simulator/simulator_robot_laser_values")