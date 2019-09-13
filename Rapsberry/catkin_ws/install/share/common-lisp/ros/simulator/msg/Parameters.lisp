; Auto-generated. Do not edit!


(cl:in-package simulator-msg)


;//! \htmlinclude Parameters.msg.html

(cl:defclass <Parameters> (roslisp-msg-protocol:ros-message)
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
   (robot_radio
    :reader robot_radio
    :initarg :robot_radio
    :type cl:float
    :initform 0.0)
   (robot_max_advance
    :reader robot_max_advance
    :initarg :robot_max_advance
    :type cl:float
    :initform 0.0)
   (robot_turn_angle
    :reader robot_turn_angle
    :initarg :robot_turn_angle
    :type cl:float
    :initform 0.0)
   (laser_num_sensors
    :reader laser_num_sensors
    :initarg :laser_num_sensors
    :type cl:integer
    :initform 0)
   (laser_origin
    :reader laser_origin
    :initarg :laser_origin
    :type cl:float
    :initform 0.0)
   (laser_range
    :reader laser_range
    :initarg :laser_range
    :type cl:float
    :initform 0.0)
   (laser_value
    :reader laser_value
    :initarg :laser_value
    :type cl:float
    :initform 0.0)
   (world_name
    :reader world_name
    :initarg :world_name
    :type cl:string
    :initform "")
   (noise
    :reader noise
    :initarg :noise
    :type cl:boolean
    :initform cl:nil)
   (run
    :reader run
    :initarg :run
    :type cl:boolean
    :initform cl:nil)
   (light_x
    :reader light_x
    :initarg :light_x
    :type cl:float
    :initform 0.0)
   (light_y
    :reader light_y
    :initarg :light_y
    :type cl:float
    :initform 0.0)
   (behavior
    :reader behavior
    :initarg :behavior
    :type cl:integer
    :initform 0)
   (steps
    :reader steps
    :initarg :steps
    :type cl:integer
    :initform 0))
)

(cl:defclass Parameters (<Parameters>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Parameters>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Parameters)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name simulator-msg:<Parameters> is deprecated: use simulator-msg:Parameters instead.")))

(cl:ensure-generic-function 'robot_x-val :lambda-list '(m))
(cl:defmethod robot_x-val ((m <Parameters>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simulator-msg:robot_x-val is deprecated.  Use simulator-msg:robot_x instead.")
  (robot_x m))

(cl:ensure-generic-function 'robot_y-val :lambda-list '(m))
(cl:defmethod robot_y-val ((m <Parameters>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simulator-msg:robot_y-val is deprecated.  Use simulator-msg:robot_y instead.")
  (robot_y m))

(cl:ensure-generic-function 'robot_theta-val :lambda-list '(m))
(cl:defmethod robot_theta-val ((m <Parameters>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simulator-msg:robot_theta-val is deprecated.  Use simulator-msg:robot_theta instead.")
  (robot_theta m))

(cl:ensure-generic-function 'robot_radio-val :lambda-list '(m))
(cl:defmethod robot_radio-val ((m <Parameters>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simulator-msg:robot_radio-val is deprecated.  Use simulator-msg:robot_radio instead.")
  (robot_radio m))

(cl:ensure-generic-function 'robot_max_advance-val :lambda-list '(m))
(cl:defmethod robot_max_advance-val ((m <Parameters>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simulator-msg:robot_max_advance-val is deprecated.  Use simulator-msg:robot_max_advance instead.")
  (robot_max_advance m))

(cl:ensure-generic-function 'robot_turn_angle-val :lambda-list '(m))
(cl:defmethod robot_turn_angle-val ((m <Parameters>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simulator-msg:robot_turn_angle-val is deprecated.  Use simulator-msg:robot_turn_angle instead.")
  (robot_turn_angle m))

(cl:ensure-generic-function 'laser_num_sensors-val :lambda-list '(m))
(cl:defmethod laser_num_sensors-val ((m <Parameters>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simulator-msg:laser_num_sensors-val is deprecated.  Use simulator-msg:laser_num_sensors instead.")
  (laser_num_sensors m))

(cl:ensure-generic-function 'laser_origin-val :lambda-list '(m))
(cl:defmethod laser_origin-val ((m <Parameters>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simulator-msg:laser_origin-val is deprecated.  Use simulator-msg:laser_origin instead.")
  (laser_origin m))

(cl:ensure-generic-function 'laser_range-val :lambda-list '(m))
(cl:defmethod laser_range-val ((m <Parameters>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simulator-msg:laser_range-val is deprecated.  Use simulator-msg:laser_range instead.")
  (laser_range m))

(cl:ensure-generic-function 'laser_value-val :lambda-list '(m))
(cl:defmethod laser_value-val ((m <Parameters>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simulator-msg:laser_value-val is deprecated.  Use simulator-msg:laser_value instead.")
  (laser_value m))

(cl:ensure-generic-function 'world_name-val :lambda-list '(m))
(cl:defmethod world_name-val ((m <Parameters>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simulator-msg:world_name-val is deprecated.  Use simulator-msg:world_name instead.")
  (world_name m))

(cl:ensure-generic-function 'noise-val :lambda-list '(m))
(cl:defmethod noise-val ((m <Parameters>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simulator-msg:noise-val is deprecated.  Use simulator-msg:noise instead.")
  (noise m))

(cl:ensure-generic-function 'run-val :lambda-list '(m))
(cl:defmethod run-val ((m <Parameters>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simulator-msg:run-val is deprecated.  Use simulator-msg:run instead.")
  (run m))

(cl:ensure-generic-function 'light_x-val :lambda-list '(m))
(cl:defmethod light_x-val ((m <Parameters>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simulator-msg:light_x-val is deprecated.  Use simulator-msg:light_x instead.")
  (light_x m))

(cl:ensure-generic-function 'light_y-val :lambda-list '(m))
(cl:defmethod light_y-val ((m <Parameters>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simulator-msg:light_y-val is deprecated.  Use simulator-msg:light_y instead.")
  (light_y m))

(cl:ensure-generic-function 'behavior-val :lambda-list '(m))
(cl:defmethod behavior-val ((m <Parameters>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simulator-msg:behavior-val is deprecated.  Use simulator-msg:behavior instead.")
  (behavior m))

(cl:ensure-generic-function 'steps-val :lambda-list '(m))
(cl:defmethod steps-val ((m <Parameters>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simulator-msg:steps-val is deprecated.  Use simulator-msg:steps instead.")
  (steps m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Parameters>) ostream)
  "Serializes a message object of type '<Parameters>"
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
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'robot_radio))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'robot_max_advance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'robot_turn_angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'laser_num_sensors)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'laser_origin))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'laser_range))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'laser_value))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'world_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'world_name))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'noise) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'run) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'light_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'light_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'behavior)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'steps)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Parameters>) istream)
  "Deserializes a message object of type '<Parameters>"
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
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'robot_radio) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'robot_max_advance) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'robot_turn_angle) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'laser_num_sensors) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'laser_origin) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'laser_range) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'laser_value) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'world_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'world_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:slot-value msg 'noise) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'run) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'light_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'light_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'behavior) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'steps) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Parameters>)))
  "Returns string type for a message object of type '<Parameters>"
  "simulator/Parameters")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Parameters)))
  "Returns string type for a message object of type 'Parameters"
  "simulator/Parameters")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Parameters>)))
  "Returns md5sum for a message object of type '<Parameters>"
  "c3a3242b503b35f6fa2f7b2245605401")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Parameters)))
  "Returns md5sum for a message object of type 'Parameters"
  "c3a3242b503b35f6fa2f7b2245605401")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Parameters>)))
  "Returns full string definition for message of type '<Parameters>"
  (cl:format cl:nil "float32 robot_x~%float32 robot_y~%float32 robot_theta~%float32 robot_radio~%float32 robot_max_advance~%float32 robot_turn_angle~%int32 laser_num_sensors~%float32 laser_origin~%float32 laser_range~%float32 laser_value~%string world_name~%bool noise~%bool run~%float32 light_x~%float32 light_y~%int32 behavior~%int32 steps~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Parameters)))
  "Returns full string definition for message of type 'Parameters"
  (cl:format cl:nil "float32 robot_x~%float32 robot_y~%float32 robot_theta~%float32 robot_radio~%float32 robot_max_advance~%float32 robot_turn_angle~%int32 laser_num_sensors~%float32 laser_origin~%float32 laser_range~%float32 laser_value~%string world_name~%bool noise~%bool run~%float32 light_x~%float32 light_y~%int32 behavior~%int32 steps~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Parameters>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4 (cl:length (cl:slot-value msg 'world_name))
     1
     1
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Parameters>))
  "Converts a ROS message object to a list"
  (cl:list 'Parameters
    (cl:cons ':robot_x (robot_x msg))
    (cl:cons ':robot_y (robot_y msg))
    (cl:cons ':robot_theta (robot_theta msg))
    (cl:cons ':robot_radio (robot_radio msg))
    (cl:cons ':robot_max_advance (robot_max_advance msg))
    (cl:cons ':robot_turn_angle (robot_turn_angle msg))
    (cl:cons ':laser_num_sensors (laser_num_sensors msg))
    (cl:cons ':laser_origin (laser_origin msg))
    (cl:cons ':laser_range (laser_range msg))
    (cl:cons ':laser_value (laser_value msg))
    (cl:cons ':world_name (world_name msg))
    (cl:cons ':noise (noise msg))
    (cl:cons ':run (run msg))
    (cl:cons ':light_x (light_x msg))
    (cl:cons ':light_y (light_y msg))
    (cl:cons ':behavior (behavior msg))
    (cl:cons ':steps (steps msg))
))
