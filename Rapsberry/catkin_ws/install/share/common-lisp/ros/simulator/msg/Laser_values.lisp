; Auto-generated. Do not edit!


(cl:in-package simulator-msg)


;//! \htmlinclude Laser_values.msg.html

(cl:defclass <Laser_values> (roslisp-msg-protocol:ros-message)
  ((sensors
    :reader sensors
    :initarg :sensors
    :type (cl:vector cl:float)
   :initform (cl:make-array 100 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass Laser_values (<Laser_values>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Laser_values>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Laser_values)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name simulator-msg:<Laser_values> is deprecated: use simulator-msg:Laser_values instead.")))

(cl:ensure-generic-function 'sensors-val :lambda-list '(m))
(cl:defmethod sensors-val ((m <Laser_values>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simulator-msg:sensors-val is deprecated.  Use simulator-msg:sensors instead.")
  (sensors m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Laser_values>) ostream)
  "Serializes a message object of type '<Laser_values>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'sensors))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Laser_values>) istream)
  "Deserializes a message object of type '<Laser_values>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Laser_values>)))
  "Returns string type for a message object of type '<Laser_values>"
  "simulator/Laser_values")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Laser_values)))
  "Returns string type for a message object of type 'Laser_values"
  "simulator/Laser_values")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Laser_values>)))
  "Returns md5sum for a message object of type '<Laser_values>"
  "e419bb63c59113a3a33a33958511350f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Laser_values)))
  "Returns md5sum for a message object of type 'Laser_values"
  "e419bb63c59113a3a33a33958511350f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Laser_values>)))
  "Returns full string definition for message of type '<Laser_values>"
  (cl:format cl:nil "float32[100] sensors~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Laser_values)))
  "Returns full string definition for message of type 'Laser_values"
  (cl:format cl:nil "float32[100] sensors~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Laser_values>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'sensors) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Laser_values>))
  "Converts a ROS message object to a list"
  (cl:list 'Laser_values
    (cl:cons ':sensors (sensors msg))
))
