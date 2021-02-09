; Auto-generated. Do not edit!


(cl:in-package ridgeback_msgs-msg)


;//! \htmlinclude Fans.msg.html

(cl:defclass <Fans> (roslisp-msg-protocol:ros-message)
  ((fans
    :reader fans
    :initarg :fans
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 6 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass Fans (<Fans>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Fans>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Fans)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ridgeback_msgs-msg:<Fans> is deprecated: use ridgeback_msgs-msg:Fans instead.")))

(cl:ensure-generic-function 'fans-val :lambda-list '(m))
(cl:defmethod fans-val ((m <Fans>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ridgeback_msgs-msg:fans-val is deprecated.  Use ridgeback_msgs-msg:fans instead.")
  (fans m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<Fans>)))
    "Constants for message type '<Fans>"
  '((:EQUIPMENT_BAY_INTAKE . 0)
    (:EQUIPMENT_BAY_EXHAUST . 1)
    (:CHARGER_BAY_INTAKE . 2)
    (:CHARGER_BAY_EXHAUST . 3)
    (:USER_BAY_INTAKE . 4)
    (:USER_BAY_EXHAUST . 5)
    (:FAN_OFF . 0)
    (:FAN_ON_HIGH . 1)
    (:FAN_ON_LOW . 2))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'Fans)))
    "Constants for message type 'Fans"
  '((:EQUIPMENT_BAY_INTAKE . 0)
    (:EQUIPMENT_BAY_EXHAUST . 1)
    (:CHARGER_BAY_INTAKE . 2)
    (:CHARGER_BAY_EXHAUST . 3)
    (:USER_BAY_INTAKE . 4)
    (:USER_BAY_EXHAUST . 5)
    (:FAN_OFF . 0)
    (:FAN_ON_HIGH . 1)
    (:FAN_ON_LOW . 2))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Fans>) ostream)
  "Serializes a message object of type '<Fans>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'fans))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Fans>) istream)
  "Deserializes a message object of type '<Fans>"
  (cl:setf (cl:slot-value msg 'fans) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'fans)))
    (cl:dotimes (i 6)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Fans>)))
  "Returns string type for a message object of type '<Fans>"
  "ridgeback_msgs/Fans")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Fans)))
  "Returns string type for a message object of type 'Fans"
  "ridgeback_msgs/Fans")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Fans>)))
  "Returns md5sum for a message object of type '<Fans>"
  "d529aec610975f8df12d912730064bbf")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Fans)))
  "Returns md5sum for a message object of type 'Fans"
  "d529aec610975f8df12d912730064bbf")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Fans>)))
  "Returns full string definition for message of type '<Fans>"
  (cl:format cl:nil "# Location of fans.~%uint8 EQUIPMENT_BAY_INTAKE=0~%uint8 EQUIPMENT_BAY_EXHAUST=1~%uint8 CHARGER_BAY_INTAKE=2~%uint8 CHARGER_BAY_EXHAUST=3~%uint8 USER_BAY_INTAKE=4~%uint8 USER_BAY_EXHAUST=5~%~%uint8 FAN_OFF=0~%uint8 FAN_ON_HIGH=1~%uint8 FAN_ON_LOW=2~%uint8[6] fans~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Fans)))
  "Returns full string definition for message of type 'Fans"
  (cl:format cl:nil "# Location of fans.~%uint8 EQUIPMENT_BAY_INTAKE=0~%uint8 EQUIPMENT_BAY_EXHAUST=1~%uint8 CHARGER_BAY_INTAKE=2~%uint8 CHARGER_BAY_EXHAUST=3~%uint8 USER_BAY_INTAKE=4~%uint8 USER_BAY_EXHAUST=5~%~%uint8 FAN_OFF=0~%uint8 FAN_ON_HIGH=1~%uint8 FAN_ON_LOW=2~%uint8[6] fans~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Fans>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'fans) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Fans>))
  "Converts a ROS message object to a list"
  (cl:list 'Fans
    (cl:cons ':fans (fans msg))
))
