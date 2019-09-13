; Auto-generated. Do not edit!


(cl:in-package simulator-srv)


;//! \htmlinclude simulator_light-request.msg.html

(cl:defclass <simulator_light-request> (roslisp-msg-protocol:ros-message)
  ((req
    :reader req
    :initarg :req
    :type cl:integer
    :initform 0))
)

(cl:defclass simulator_light-request (<simulator_light-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <simulator_light-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'simulator_light-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name simulator-srv:<simulator_light-request> is deprecated: use simulator-srv:simulator_light-request instead.")))

(cl:ensure-generic-function 'req-val :lambda-list '(m))
(cl:defmethod req-val ((m <simulator_light-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simulator-srv:req-val is deprecated.  Use simulator-srv:req instead.")
  (req m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <simulator_light-request>) ostream)
  "Serializes a message object of type '<simulator_light-request>"
  (cl:let* ((signed (cl:slot-value msg 'req)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <simulator_light-request>) istream)
  "Deserializes a message object of type '<simulator_light-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'req) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<simulator_light-request>)))
  "Returns string type for a service object of type '<simulator_light-request>"
  "simulator/simulator_lightRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'simulator_light-request)))
  "Returns string type for a service object of type 'simulator_light-request"
  "simulator/simulator_lightRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<simulator_light-request>)))
  "Returns md5sum for a message object of type '<simulator_light-request>"
  "f0a363bdf6e9db5ed005776d88d3f68d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'simulator_light-request)))
  "Returns md5sum for a message object of type 'simulator_light-request"
  "f0a363bdf6e9db5ed005776d88d3f68d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<simulator_light-request>)))
  "Returns full string definition for message of type '<simulator_light-request>"
  (cl:format cl:nil "int32 req~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'simulator_light-request)))
  "Returns full string definition for message of type 'simulator_light-request"
  (cl:format cl:nil "int32 req~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <simulator_light-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <simulator_light-request>))
  "Converts a ROS message object to a list"
  (cl:list 'simulator_light-request
    (cl:cons ':req (req msg))
))
;//! \htmlinclude simulator_light-response.msg.html

(cl:defclass <simulator_light-response> (roslisp-msg-protocol:ros-message)
  ((values
    :reader values
    :initarg :values
    :type (cl:vector cl:float)
   :initform (cl:make-array 8 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass simulator_light-response (<simulator_light-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <simulator_light-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'simulator_light-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name simulator-srv:<simulator_light-response> is deprecated: use simulator-srv:simulator_light-response instead.")))

(cl:ensure-generic-function 'values-val :lambda-list '(m))
(cl:defmethod values-val ((m <simulator_light-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simulator-srv:values-val is deprecated.  Use simulator-srv:values instead.")
  (values m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <simulator_light-response>) ostream)
  "Serializes a message object of type '<simulator_light-response>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'values))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <simulator_light-response>) istream)
  "Deserializes a message object of type '<simulator_light-response>"
  (cl:setf (cl:slot-value msg 'values) (cl:make-array 8))
  (cl:let ((vals (cl:slot-value msg 'values)))
    (cl:dotimes (i 8)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<simulator_light-response>)))
  "Returns string type for a service object of type '<simulator_light-response>"
  "simulator/simulator_lightResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'simulator_light-response)))
  "Returns string type for a service object of type 'simulator_light-response"
  "simulator/simulator_lightResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<simulator_light-response>)))
  "Returns md5sum for a message object of type '<simulator_light-response>"
  "f0a363bdf6e9db5ed005776d88d3f68d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'simulator_light-response)))
  "Returns md5sum for a message object of type 'simulator_light-response"
  "f0a363bdf6e9db5ed005776d88d3f68d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<simulator_light-response>)))
  "Returns full string definition for message of type '<simulator_light-response>"
  (cl:format cl:nil "float32[8] values~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'simulator_light-response)))
  "Returns full string definition for message of type 'simulator_light-response"
  (cl:format cl:nil "float32[8] values~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <simulator_light-response>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'values) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <simulator_light-response>))
  "Converts a ROS message object to a list"
  (cl:list 'simulator_light-response
    (cl:cons ':values (values msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'simulator_light)))
  'simulator_light-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'simulator_light)))
  'simulator_light-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'simulator_light)))
  "Returns string type for a service object of type '<simulator_light>"
  "simulator/simulator_light")