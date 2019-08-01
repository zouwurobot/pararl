; Auto-generated. Do not edit!


(cl:in-package msgs-srv)


;//! \htmlinclude Home-request.msg.html

(cl:defclass <Home-request> (roslisp-msg-protocol:ros-message)
  ((home
    :reader home
    :initarg :home
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Home-request (<Home-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Home-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Home-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name msgs-srv:<Home-request> is deprecated: use msgs-srv:Home-request instead.")))

(cl:ensure-generic-function 'home-val :lambda-list '(m))
(cl:defmethod home-val ((m <Home-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msgs-srv:home-val is deprecated.  Use msgs-srv:home instead.")
  (home m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Home-request>) ostream)
  "Serializes a message object of type '<Home-request>"
  (cl:let* ((signed (cl:slot-value msg 'home)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Home-request>) istream)
  "Deserializes a message object of type '<Home-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'home) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Home-request>)))
  "Returns string type for a service object of type '<Home-request>"
  "msgs/HomeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Home-request)))
  "Returns string type for a service object of type 'Home-request"
  "msgs/HomeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Home-request>)))
  "Returns md5sum for a message object of type '<Home-request>"
  "315d885f8f6e90abb232f2881df249a2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Home-request)))
  "Returns md5sum for a message object of type 'Home-request"
  "315d885f8f6e90abb232f2881df249a2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Home-request>)))
  "Returns full string definition for message of type '<Home-request>"
  (cl:format cl:nil "int8 home~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Home-request)))
  "Returns full string definition for message of type 'Home-request"
  (cl:format cl:nil "int8 home~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Home-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Home-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Home-request
    (cl:cons ':home (home msg))
))
;//! \htmlinclude Home-response.msg.html

(cl:defclass <Home-response> (roslisp-msg-protocol:ros-message)
  ((done
    :reader done
    :initarg :done
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Home-response (<Home-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Home-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Home-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name msgs-srv:<Home-response> is deprecated: use msgs-srv:Home-response instead.")))

(cl:ensure-generic-function 'done-val :lambda-list '(m))
(cl:defmethod done-val ((m <Home-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msgs-srv:done-val is deprecated.  Use msgs-srv:done instead.")
  (done m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Home-response>) ostream)
  "Serializes a message object of type '<Home-response>"
  (cl:let* ((signed (cl:slot-value msg 'done)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Home-response>) istream)
  "Deserializes a message object of type '<Home-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'done) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Home-response>)))
  "Returns string type for a service object of type '<Home-response>"
  "msgs/HomeResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Home-response)))
  "Returns string type for a service object of type 'Home-response"
  "msgs/HomeResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Home-response>)))
  "Returns md5sum for a message object of type '<Home-response>"
  "315d885f8f6e90abb232f2881df249a2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Home-response)))
  "Returns md5sum for a message object of type 'Home-response"
  "315d885f8f6e90abb232f2881df249a2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Home-response>)))
  "Returns full string definition for message of type '<Home-response>"
  (cl:format cl:nil "int8 done~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Home-response)))
  "Returns full string definition for message of type 'Home-response"
  (cl:format cl:nil "int8 done~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Home-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Home-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Home-response
    (cl:cons ':done (done msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Home)))
  'Home-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Home)))
  'Home-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Home)))
  "Returns string type for a service object of type '<Home>"
  "msgs/Home")