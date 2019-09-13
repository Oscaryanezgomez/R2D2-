; Auto-generated. Do not edit!


(cl:in-package simulator-srv)


;//! \htmlinclude simulator_algorithm_result-request.msg.html

(cl:defclass <simulator_algorithm_result-request> (roslisp-msg-protocol:ros-message)
  ((nodes_algorithm
    :reader nodes_algorithm
    :initarg :nodes_algorithm
    :type (cl:vector cl:integer)
   :initform (cl:make-array 200 :element-type 'cl:integer :initial-element 0))
   (num_nodos
    :reader num_nodos
    :initarg :num_nodos
    :type cl:integer
    :initform 0))
)

(cl:defclass simulator_algorithm_result-request (<simulator_algorithm_result-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <simulator_algorithm_result-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'simulator_algorithm_result-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name simulator-srv:<simulator_algorithm_result-request> is deprecated: use simulator-srv:simulator_algorithm_result-request instead.")))

(cl:ensure-generic-function 'nodes_algorithm-val :lambda-list '(m))
(cl:defmethod nodes_algorithm-val ((m <simulator_algorithm_result-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simulator-srv:nodes_algorithm-val is deprecated.  Use simulator-srv:nodes_algorithm instead.")
  (nodes_algorithm m))

(cl:ensure-generic-function 'num_nodos-val :lambda-list '(m))
(cl:defmethod num_nodos-val ((m <simulator_algorithm_result-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simulator-srv:num_nodos-val is deprecated.  Use simulator-srv:num_nodos instead.")
  (num_nodos m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <simulator_algorithm_result-request>) ostream)
  "Serializes a message object of type '<simulator_algorithm_result-request>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'nodes_algorithm))
  (cl:let* ((signed (cl:slot-value msg 'num_nodos)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <simulator_algorithm_result-request>) istream)
  "Deserializes a message object of type '<simulator_algorithm_result-request>"
  (cl:setf (cl:slot-value msg 'nodes_algorithm) (cl:make-array 200))
  (cl:let ((vals (cl:slot-value msg 'nodes_algorithm)))
    (cl:dotimes (i 200)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'num_nodos) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<simulator_algorithm_result-request>)))
  "Returns string type for a service object of type '<simulator_algorithm_result-request>"
  "simulator/simulator_algorithm_resultRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'simulator_algorithm_result-request)))
  "Returns string type for a service object of type 'simulator_algorithm_result-request"
  "simulator/simulator_algorithm_resultRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<simulator_algorithm_result-request>)))
  "Returns md5sum for a message object of type '<simulator_algorithm_result-request>"
  "d56e85175467db2ddcdf7fd7b9140749")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'simulator_algorithm_result-request)))
  "Returns md5sum for a message object of type 'simulator_algorithm_result-request"
  "d56e85175467db2ddcdf7fd7b9140749")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<simulator_algorithm_result-request>)))
  "Returns full string definition for message of type '<simulator_algorithm_result-request>"
  (cl:format cl:nil "int32[200] nodes_algorithm~%int32 num_nodos~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'simulator_algorithm_result-request)))
  "Returns full string definition for message of type 'simulator_algorithm_result-request"
  (cl:format cl:nil "int32[200] nodes_algorithm~%int32 num_nodos~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <simulator_algorithm_result-request>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'nodes_algorithm) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <simulator_algorithm_result-request>))
  "Converts a ROS message object to a list"
  (cl:list 'simulator_algorithm_result-request
    (cl:cons ':nodes_algorithm (nodes_algorithm msg))
    (cl:cons ':num_nodos (num_nodos msg))
))
;//! \htmlinclude simulator_algorithm_result-response.msg.html

(cl:defclass <simulator_algorithm_result-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:integer
    :initform 0))
)

(cl:defclass simulator_algorithm_result-response (<simulator_algorithm_result-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <simulator_algorithm_result-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'simulator_algorithm_result-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name simulator-srv:<simulator_algorithm_result-response> is deprecated: use simulator-srv:simulator_algorithm_result-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <simulator_algorithm_result-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simulator-srv:success-val is deprecated.  Use simulator-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <simulator_algorithm_result-response>) ostream)
  "Serializes a message object of type '<simulator_algorithm_result-response>"
  (cl:let* ((signed (cl:slot-value msg 'success)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <simulator_algorithm_result-response>) istream)
  "Deserializes a message object of type '<simulator_algorithm_result-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'success) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<simulator_algorithm_result-response>)))
  "Returns string type for a service object of type '<simulator_algorithm_result-response>"
  "simulator/simulator_algorithm_resultResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'simulator_algorithm_result-response)))
  "Returns string type for a service object of type 'simulator_algorithm_result-response"
  "simulator/simulator_algorithm_resultResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<simulator_algorithm_result-response>)))
  "Returns md5sum for a message object of type '<simulator_algorithm_result-response>"
  "d56e85175467db2ddcdf7fd7b9140749")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'simulator_algorithm_result-response)))
  "Returns md5sum for a message object of type 'simulator_algorithm_result-response"
  "d56e85175467db2ddcdf7fd7b9140749")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<simulator_algorithm_result-response>)))
  "Returns full string definition for message of type '<simulator_algorithm_result-response>"
  (cl:format cl:nil "int32 success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'simulator_algorithm_result-response)))
  "Returns full string definition for message of type 'simulator_algorithm_result-response"
  (cl:format cl:nil "int32 success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <simulator_algorithm_result-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <simulator_algorithm_result-response>))
  "Converts a ROS message object to a list"
  (cl:list 'simulator_algorithm_result-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'simulator_algorithm_result)))
  'simulator_algorithm_result-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'simulator_algorithm_result)))
  'simulator_algorithm_result-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'simulator_algorithm_result)))
  "Returns string type for a service object of type '<simulator_algorithm_result>"
  "simulator/simulator_algorithm_result")