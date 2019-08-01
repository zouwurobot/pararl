; Auto-generated. Do not edit!


(cl:in-package msgs-srv)


;//! \htmlinclude HomeAndLimit-request.msg.html

(cl:defclass <HomeAndLimit-request> (roslisp-msg-protocol:ros-message)
  ((home_xyz
    :reader home_xyz
    :initarg :home_xyz
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0))
   (home_oreintation
    :reader home_oreintation
    :initarg :home_oreintation
    :type (cl:vector cl:float)
   :initform (cl:make-array 4 :element-type 'cl:float :initial-element 0.0))
   (limit_range
    :reader limit_range
    :initarg :limit_range
    :type (cl:vector cl:float)
   :initform (cl:make-array 6 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass HomeAndLimit-request (<HomeAndLimit-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <HomeAndLimit-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'HomeAndLimit-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name msgs-srv:<HomeAndLimit-request> is deprecated: use msgs-srv:HomeAndLimit-request instead.")))

(cl:ensure-generic-function 'home_xyz-val :lambda-list '(m))
(cl:defmethod home_xyz-val ((m <HomeAndLimit-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msgs-srv:home_xyz-val is deprecated.  Use msgs-srv:home_xyz instead.")
  (home_xyz m))

(cl:ensure-generic-function 'home_oreintation-val :lambda-list '(m))
(cl:defmethod home_oreintation-val ((m <HomeAndLimit-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msgs-srv:home_oreintation-val is deprecated.  Use msgs-srv:home_oreintation instead.")
  (home_oreintation m))

(cl:ensure-generic-function 'limit_range-val :lambda-list '(m))
(cl:defmethod limit_range-val ((m <HomeAndLimit-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msgs-srv:limit_range-val is deprecated.  Use msgs-srv:limit_range instead.")
  (limit_range m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <HomeAndLimit-request>) ostream)
  "Serializes a message object of type '<HomeAndLimit-request>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'home_xyz))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'home_oreintation))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'limit_range))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <HomeAndLimit-request>) istream)
  "Deserializes a message object of type '<HomeAndLimit-request>"
  (cl:setf (cl:slot-value msg 'home_xyz) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'home_xyz)))
    (cl:dotimes (i 3)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'home_oreintation) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'home_oreintation)))
    (cl:dotimes (i 4)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'limit_range) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'limit_range)))
    (cl:dotimes (i 6)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<HomeAndLimit-request>)))
  "Returns string type for a service object of type '<HomeAndLimit-request>"
  "msgs/HomeAndLimitRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HomeAndLimit-request)))
  "Returns string type for a service object of type 'HomeAndLimit-request"
  "msgs/HomeAndLimitRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<HomeAndLimit-request>)))
  "Returns md5sum for a message object of type '<HomeAndLimit-request>"
  "37986a4fb94d2bd1e620d61c5fcb7476")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'HomeAndLimit-request)))
  "Returns md5sum for a message object of type 'HomeAndLimit-request"
  "37986a4fb94d2bd1e620d61c5fcb7476")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<HomeAndLimit-request>)))
  "Returns full string definition for message of type '<HomeAndLimit-request>"
  (cl:format cl:nil "float64[3] home_xyz~%float64[4] home_oreintation~%float64[6] limit_range~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'HomeAndLimit-request)))
  "Returns full string definition for message of type 'HomeAndLimit-request"
  (cl:format cl:nil "float64[3] home_xyz~%float64[4] home_oreintation~%float64[6] limit_range~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <HomeAndLimit-request>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'home_xyz) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'home_oreintation) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'limit_range) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <HomeAndLimit-request>))
  "Converts a ROS message object to a list"
  (cl:list 'HomeAndLimit-request
    (cl:cons ':home_xyz (home_xyz msg))
    (cl:cons ':home_oreintation (home_oreintation msg))
    (cl:cons ':limit_range (limit_range msg))
))
;//! \htmlinclude HomeAndLimit-response.msg.html

(cl:defclass <HomeAndLimit-response> (roslisp-msg-protocol:ros-message)
  ((done
    :reader done
    :initarg :done
    :type cl:fixnum
    :initform 0))
)

(cl:defclass HomeAndLimit-response (<HomeAndLimit-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <HomeAndLimit-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'HomeAndLimit-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name msgs-srv:<HomeAndLimit-response> is deprecated: use msgs-srv:HomeAndLimit-response instead.")))

(cl:ensure-generic-function 'done-val :lambda-list '(m))
(cl:defmethod done-val ((m <HomeAndLimit-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msgs-srv:done-val is deprecated.  Use msgs-srv:done instead.")
  (done m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <HomeAndLimit-response>) ostream)
  "Serializes a message object of type '<HomeAndLimit-response>"
  (cl:let* ((signed (cl:slot-value msg 'done)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <HomeAndLimit-response>) istream)
  "Deserializes a message object of type '<HomeAndLimit-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'done) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<HomeAndLimit-response>)))
  "Returns string type for a service object of type '<HomeAndLimit-response>"
  "msgs/HomeAndLimitResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HomeAndLimit-response)))
  "Returns string type for a service object of type 'HomeAndLimit-response"
  "msgs/HomeAndLimitResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<HomeAndLimit-response>)))
  "Returns md5sum for a message object of type '<HomeAndLimit-response>"
  "37986a4fb94d2bd1e620d61c5fcb7476")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'HomeAndLimit-response)))
  "Returns md5sum for a message object of type 'HomeAndLimit-response"
  "37986a4fb94d2bd1e620d61c5fcb7476")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<HomeAndLimit-response>)))
  "Returns full string definition for message of type '<HomeAndLimit-response>"
  (cl:format cl:nil "int8 done~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'HomeAndLimit-response)))
  "Returns full string definition for message of type 'HomeAndLimit-response"
  (cl:format cl:nil "int8 done~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <HomeAndLimit-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <HomeAndLimit-response>))
  "Converts a ROS message object to a list"
  (cl:list 'HomeAndLimit-response
    (cl:cons ':done (done msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'HomeAndLimit)))
  'HomeAndLimit-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'HomeAndLimit)))
  'HomeAndLimit-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HomeAndLimit)))
  "Returns string type for a service object of type '<HomeAndLimit>"
  "msgs/HomeAndLimit")