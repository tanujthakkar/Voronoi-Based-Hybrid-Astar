; Auto-generated. Do not edit!


(cl:in-package ridgeback_msgs-msg)


;//! \htmlinclude RGB.msg.html

(cl:defclass <RGB> (roslisp-msg-protocol:ros-message)
  ((red
    :reader red
    :initarg :red
    :type cl:float
    :initform 0.0)
   (green
    :reader green
    :initarg :green
    :type cl:float
    :initform 0.0)
   (blue
    :reader blue
    :initarg :blue
    :type cl:float
    :initform 0.0))
)

(cl:defclass RGB (<RGB>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RGB>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RGB)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ridgeback_msgs-msg:<RGB> is deprecated: use ridgeback_msgs-msg:RGB instead.")))

(cl:ensure-generic-function 'red-val :lambda-list '(m))
(cl:defmethod red-val ((m <RGB>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ridgeback_msgs-msg:red-val is deprecated.  Use ridgeback_msgs-msg:red instead.")
  (red m))

(cl:ensure-generic-function 'green-val :lambda-list '(m))
(cl:defmethod green-val ((m <RGB>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ridgeback_msgs-msg:green-val is deprecated.  Use ridgeback_msgs-msg:green instead.")
  (green m))

(cl:ensure-generic-function 'blue-val :lambda-list '(m))
(cl:defmethod blue-val ((m <RGB>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ridgeback_msgs-msg:blue-val is deprecated.  Use ridgeback_msgs-msg:blue instead.")
  (blue m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RGB>) ostream)
  "Serializes a message object of type '<RGB>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'red))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'green))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'blue))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RGB>) istream)
  "Deserializes a message object of type '<RGB>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'red) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'green) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'blue) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RGB>)))
  "Returns string type for a message object of type '<RGB>"
  "ridgeback_msgs/RGB")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RGB)))
  "Returns string type for a message object of type 'RGB"
  "ridgeback_msgs/RGB")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RGB>)))
  "Returns md5sum for a message object of type '<RGB>"
  "fc84fca2ee69069d6d5c4147f9b2e33a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RGB)))
  "Returns md5sum for a message object of type 'RGB"
  "fc84fca2ee69069d6d5c4147f9b2e33a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RGB>)))
  "Returns full string definition for message of type '<RGB>"
  (cl:format cl:nil "# Represents the intensity of a single RGB LED, either reported or commanded.~%~%float32 red~%float32 green~%float32 blue~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RGB)))
  "Returns full string definition for message of type 'RGB"
  (cl:format cl:nil "# Represents the intensity of a single RGB LED, either reported or commanded.~%~%float32 red~%float32 green~%float32 blue~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RGB>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RGB>))
  "Converts a ROS message object to a list"
  (cl:list 'RGB
    (cl:cons ':red (red msg))
    (cl:cons ':green (green msg))
    (cl:cons ':blue (blue msg))
))
