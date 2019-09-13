; Auto-generated. Do not edit!


(cl:in-package simulator-srv)


;//! \htmlinclude simulator_laser-request.msg.html

(cl:defclass <simulator_laser-request> (roslisp-msg-protocol:ros-message)
  ((robot_x
    :reader robot_x
    :initarg :robot_x
    :type cl:float
    :initform 0.0)
   (robot_y
    :reader robot_y
    :initarg :robot_y
    :type cl:float
    :initform 0.0)
   (robot_theta
    :reader robot_theta
    :initarg :robot_theta
    :type cl:float
    :initform 0.0)
   (new_simulation
    :reader new_simulation
    :initarg :new_simulation
    :type cl:integer
    :initform 0))
)

(cl:defclass simulator_laser-request (<simulator_laser-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <simulator_laser-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'simulator_laser-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name simulator-srv:<simulator_laser-request> is deprecated: use simulator-srv:simulator_laser-request instead.")))

(cl:ensure-generic-function 'robot_x-val :lambda-list '(m))
(cl:defmethod robot_x-val ((m <simulator_laser-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simulator-srv:robot_x-val is deprecated.  Use simulator-srv:robot_x instead.")
  (robot_x m))

(cl:ensure-generic-function 'robot_y-val :lambda-list '(m))
(cl:defmethod robot_y-val ((m <simulator_laser-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simulator-srv:robot_y-val is deprecated.  Use simulator-srv:robot_y instead.")
  (robot_y m))

(cl:ensure-generic-function 'robot_theta-val :lambda-list '(m))
(cl:defmethod robot_theta-val ((m <simulator_laser-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simulator-srv:robot_theta-val is deprecated.  Use simulator-srv:robot_theta instead.")
  (robot_theta m))

(cl:ensure-generic-function 'new_simulation-val :lambda-list '(m))
(cl:defmethod new_simulation-val ((m <simulator_laser-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simulator-srv:new_simulation-val is deprecated.  Use simulator-srv:new_simulation instead.")
  (new_simulation m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <simulator_laser-request>) ostream)
  "Serializes a message object of type '<simulator_laser-request>"
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
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'robot_theta))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'new_simulation)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <simulator_laser-request>) istream)
  "Deserializes a message object of type '<simulator_laser-request>"
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
    (cl:setf (cl:slot-value msg 'robot_theta) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'new_simulation) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<simulator_laser-request>)))
  "Returns string type for a service object of type '<simulator_laser-request>"
  "simulator/simulator_laserRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'simulator_laser-request)))
  "Returns string type for a service object of type 'simulator_laser-request"
  "simulator/simulator_laserRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<simulator_laser-request>)))
  "Returns md5sum for a message object of type '<simulator_laser-request>"
  "411061852dc5f8a41040799d09b07364")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'simulator_laser-request)))
  "Returns md5sum for a message object of type 'simulator_laser-request"
  "411061852dc5f8a41040799d09b07364")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<simulator_laser-request>)))
  "Returns full string definition for message of type '<simulator_laser-request>"
  (cl:format cl:nil "float32 robot_x~%float32 robot_y~%float32 robot_theta~%int32 new_simulation~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'simulator_laser-request)))
  "Returns full string definition for message of type 'simulator_laser-request"
  (cl:format cl:nil "float32 robot_x~%float32 robot_y~%float32 robot_theta~%int32 new_simulation~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <simulator_laser-request>))
  (cl:+ 0
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <simulator_laser-request>))
  "Converts a ROS message object to a list"
  (cl:list 'simulator_laser-request
    (cl:cons ':robot_x (robot_x msg))
    (cl:cons ':robot_y (robot_y msg))
    (cl:cons ':robot_theta (robot_theta msg))
    (cl:cons ':new_simulation (new_simulation msg))
))
;//! \htmlinclude simulator_laser-response.msg.html

(cl:defclass <simulator_laser-response> (roslisp-msg-protocol:ros-message)
  ((sensors
    :reader sensors
    :initarg :sensors
    :type (cl:vector cl:float)
   :initform (cl:make-array 100 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass simulator_laser-response (<simulator_laser-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <simulator_laser-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'simulator_laser-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name simulator-srv:<simulator_laser-response> is deprecated: use simulator-srv:simulator_laser-response instead.")))

(cl:ensure-generic-function 'sensors-val :lambda-list '(m))
(cl:defmethod sensors-val ((m <simulator_laser-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simulator-srv:sensors-val is deprecated.  Use simulator-srv:sensors instead.")
  (sensors m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <simulator_laser-response>) ostream)
  "Serializes a message object of type '<simulator_laser-response>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'sensors))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <simulator_laser-response>) istream)
  "Deserializes a message object of type '<simulator_laser-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<simulator_laser-response>)))
  "Returns string type for a service object of type '<simulator_laser-response>"
  "simulator/simulator_laserResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'simulator_laser-response)))
  "Returns string type for a service object of type 'simulator_laser-response"
  "simulator/simulator_laserResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<simulator_laser-response>)))
  "Returns md5sum for a message object of type '<simulator_laser-response>"
  "411061852dc5f8a41040799d09b07364")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'simulator_laser-response)))
  "Returns md5sum for a message object of type 'simulator_laser-response"
  "411061852dc5f8a41040799d09b07364")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<simulator_laser-response>)))
  "Returns full string definition for message of type '<simulator_laser-response>"
  (cl:format cl:nil "float32[100] sensors~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'simulator_laser-response)))
  "Returns full string definition for message of type 'simulator_laser-response"
  (cl:format cl:nil "float32[100] sensors~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <simulator_laser-response>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'sensors) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <simulator_laser-response>))
  "Converts a ROS message object to a list"
  (cl:list 'simulator_laser-response
    (cl:cons ':sensors (sensors msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'simulator_laser)))
  'simulator_laser-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'simulator_laser)))
  'simulator_laser-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'simulator_laser)))
  "Returns string type for a service object of type '<simulator_laser>"
  "simulator/simulator_laser")