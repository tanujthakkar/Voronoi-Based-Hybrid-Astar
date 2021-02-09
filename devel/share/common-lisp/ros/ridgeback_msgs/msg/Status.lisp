; Auto-generated. Do not edit!


(cl:in-package ridgeback_msgs-msg)


;//! \htmlinclude Status.msg.html

(cl:defclass <Status> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (hardware_id
    :reader hardware_id
    :initarg :hardware_id
    :type cl:string
    :initform "")
   (mcu_uptime
    :reader mcu_uptime
    :initarg :mcu_uptime
    :type cl:real
    :initform 0)
   (connection_uptime
    :reader connection_uptime
    :initarg :connection_uptime
    :type cl:real
    :initform 0)
   (pcb_temperature
    :reader pcb_temperature
    :initarg :pcb_temperature
    :type cl:float
    :initform 0.0)
   (mcu_temperature
    :reader mcu_temperature
    :initarg :mcu_temperature
    :type cl:float
    :initform 0.0)
   (stop_power_status
    :reader stop_power_status
    :initarg :stop_power_status
    :type cl:boolean
    :initform cl:nil)
   (stop_engaged
    :reader stop_engaged
    :initarg :stop_engaged
    :type cl:boolean
    :initform cl:nil)
   (drivers_active
    :reader drivers_active
    :initarg :drivers_active
    :type cl:boolean
    :initform cl:nil)
   (external_stop_present
    :reader external_stop_present
    :initarg :external_stop_present
    :type cl:boolean
    :initform cl:nil)
   (charger_connected
    :reader charger_connected
    :initarg :charger_connected
    :type cl:boolean
    :initform cl:nil)
   (charging_complete
    :reader charging_complete
    :initarg :charging_complete
    :type cl:boolean
    :initform cl:nil)
   (measured_battery
    :reader measured_battery
    :initarg :measured_battery
    :type cl:float
    :initform 0.0)
   (measured_12v
    :reader measured_12v
    :initarg :measured_12v
    :type cl:float
    :initform 0.0)
   (measured_5v
    :reader measured_5v
    :initarg :measured_5v
    :type cl:float
    :initform 0.0)
   (measured_inverter
    :reader measured_inverter
    :initarg :measured_inverter
    :type cl:float
    :initform 0.0)
   (measured_front_axle
    :reader measured_front_axle
    :initarg :measured_front_axle
    :type cl:float
    :initform 0.0)
   (measured_rear_axle
    :reader measured_rear_axle
    :initarg :measured_rear_axle
    :type cl:float
    :initform 0.0)
   (measured_light
    :reader measured_light
    :initarg :measured_light
    :type cl:float
    :initform 0.0)
   (total_current
    :reader total_current
    :initarg :total_current
    :type cl:float
    :initform 0.0)
   (total_current_peak
    :reader total_current_peak
    :initarg :total_current_peak
    :type cl:float
    :initform 0.0)
   (total_power_consumed
    :reader total_power_consumed
    :initarg :total_power_consumed
    :type cl:float
    :initform 0.0))
)

(cl:defclass Status (<Status>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Status>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Status)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ridgeback_msgs-msg:<Status> is deprecated: use ridgeback_msgs-msg:Status instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ridgeback_msgs-msg:header-val is deprecated.  Use ridgeback_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'hardware_id-val :lambda-list '(m))
(cl:defmethod hardware_id-val ((m <Status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ridgeback_msgs-msg:hardware_id-val is deprecated.  Use ridgeback_msgs-msg:hardware_id instead.")
  (hardware_id m))

(cl:ensure-generic-function 'mcu_uptime-val :lambda-list '(m))
(cl:defmethod mcu_uptime-val ((m <Status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ridgeback_msgs-msg:mcu_uptime-val is deprecated.  Use ridgeback_msgs-msg:mcu_uptime instead.")
  (mcu_uptime m))

(cl:ensure-generic-function 'connection_uptime-val :lambda-list '(m))
(cl:defmethod connection_uptime-val ((m <Status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ridgeback_msgs-msg:connection_uptime-val is deprecated.  Use ridgeback_msgs-msg:connection_uptime instead.")
  (connection_uptime m))

(cl:ensure-generic-function 'pcb_temperature-val :lambda-list '(m))
(cl:defmethod pcb_temperature-val ((m <Status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ridgeback_msgs-msg:pcb_temperature-val is deprecated.  Use ridgeback_msgs-msg:pcb_temperature instead.")
  (pcb_temperature m))

(cl:ensure-generic-function 'mcu_temperature-val :lambda-list '(m))
(cl:defmethod mcu_temperature-val ((m <Status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ridgeback_msgs-msg:mcu_temperature-val is deprecated.  Use ridgeback_msgs-msg:mcu_temperature instead.")
  (mcu_temperature m))

(cl:ensure-generic-function 'stop_power_status-val :lambda-list '(m))
(cl:defmethod stop_power_status-val ((m <Status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ridgeback_msgs-msg:stop_power_status-val is deprecated.  Use ridgeback_msgs-msg:stop_power_status instead.")
  (stop_power_status m))

(cl:ensure-generic-function 'stop_engaged-val :lambda-list '(m))
(cl:defmethod stop_engaged-val ((m <Status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ridgeback_msgs-msg:stop_engaged-val is deprecated.  Use ridgeback_msgs-msg:stop_engaged instead.")
  (stop_engaged m))

(cl:ensure-generic-function 'drivers_active-val :lambda-list '(m))
(cl:defmethod drivers_active-val ((m <Status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ridgeback_msgs-msg:drivers_active-val is deprecated.  Use ridgeback_msgs-msg:drivers_active instead.")
  (drivers_active m))

(cl:ensure-generic-function 'external_stop_present-val :lambda-list '(m))
(cl:defmethod external_stop_present-val ((m <Status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ridgeback_msgs-msg:external_stop_present-val is deprecated.  Use ridgeback_msgs-msg:external_stop_present instead.")
  (external_stop_present m))

(cl:ensure-generic-function 'charger_connected-val :lambda-list '(m))
(cl:defmethod charger_connected-val ((m <Status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ridgeback_msgs-msg:charger_connected-val is deprecated.  Use ridgeback_msgs-msg:charger_connected instead.")
  (charger_connected m))

(cl:ensure-generic-function 'charging_complete-val :lambda-list '(m))
(cl:defmethod charging_complete-val ((m <Status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ridgeback_msgs-msg:charging_complete-val is deprecated.  Use ridgeback_msgs-msg:charging_complete instead.")
  (charging_complete m))

(cl:ensure-generic-function 'measured_battery-val :lambda-list '(m))
(cl:defmethod measured_battery-val ((m <Status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ridgeback_msgs-msg:measured_battery-val is deprecated.  Use ridgeback_msgs-msg:measured_battery instead.")
  (measured_battery m))

(cl:ensure-generic-function 'measured_12v-val :lambda-list '(m))
(cl:defmethod measured_12v-val ((m <Status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ridgeback_msgs-msg:measured_12v-val is deprecated.  Use ridgeback_msgs-msg:measured_12v instead.")
  (measured_12v m))

(cl:ensure-generic-function 'measured_5v-val :lambda-list '(m))
(cl:defmethod measured_5v-val ((m <Status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ridgeback_msgs-msg:measured_5v-val is deprecated.  Use ridgeback_msgs-msg:measured_5v instead.")
  (measured_5v m))

(cl:ensure-generic-function 'measured_inverter-val :lambda-list '(m))
(cl:defmethod measured_inverter-val ((m <Status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ridgeback_msgs-msg:measured_inverter-val is deprecated.  Use ridgeback_msgs-msg:measured_inverter instead.")
  (measured_inverter m))

(cl:ensure-generic-function 'measured_front_axle-val :lambda-list '(m))
(cl:defmethod measured_front_axle-val ((m <Status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ridgeback_msgs-msg:measured_front_axle-val is deprecated.  Use ridgeback_msgs-msg:measured_front_axle instead.")
  (measured_front_axle m))

(cl:ensure-generic-function 'measured_rear_axle-val :lambda-list '(m))
(cl:defmethod measured_rear_axle-val ((m <Status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ridgeback_msgs-msg:measured_rear_axle-val is deprecated.  Use ridgeback_msgs-msg:measured_rear_axle instead.")
  (measured_rear_axle m))

(cl:ensure-generic-function 'measured_light-val :lambda-list '(m))
(cl:defmethod measured_light-val ((m <Status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ridgeback_msgs-msg:measured_light-val is deprecated.  Use ridgeback_msgs-msg:measured_light instead.")
  (measured_light m))

(cl:ensure-generic-function 'total_current-val :lambda-list '(m))
(cl:defmethod total_current-val ((m <Status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ridgeback_msgs-msg:total_current-val is deprecated.  Use ridgeback_msgs-msg:total_current instead.")
  (total_current m))

(cl:ensure-generic-function 'total_current_peak-val :lambda-list '(m))
(cl:defmethod total_current_peak-val ((m <Status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ridgeback_msgs-msg:total_current_peak-val is deprecated.  Use ridgeback_msgs-msg:total_current_peak instead.")
  (total_current_peak m))

(cl:ensure-generic-function 'total_power_consumed-val :lambda-list '(m))
(cl:defmethod total_power_consumed-val ((m <Status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ridgeback_msgs-msg:total_power_consumed-val is deprecated.  Use ridgeback_msgs-msg:total_power_consumed instead.")
  (total_power_consumed m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Status>) ostream)
  "Serializes a message object of type '<Status>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'hardware_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'hardware_id))
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'mcu_uptime)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'mcu_uptime) (cl:floor (cl:slot-value msg 'mcu_uptime)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'connection_uptime)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'connection_uptime) (cl:floor (cl:slot-value msg 'connection_uptime)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pcb_temperature))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'mcu_temperature))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'stop_power_status) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'stop_engaged) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'drivers_active) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'external_stop_present) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'charger_connected) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'charging_complete) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'measured_battery))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'measured_12v))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'measured_5v))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'measured_inverter))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'measured_front_axle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'measured_rear_axle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'measured_light))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'total_current))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'total_current_peak))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'total_power_consumed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Status>) istream)
  "Deserializes a message object of type '<Status>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'hardware_id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'hardware_id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'mcu_uptime) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'connection_uptime) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pcb_temperature) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'mcu_temperature) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:slot-value msg 'stop_power_status) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'stop_engaged) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'drivers_active) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'external_stop_present) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'charger_connected) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'charging_complete) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'measured_battery) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'measured_12v) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'measured_5v) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'measured_inverter) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'measured_front_axle) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'measured_rear_axle) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'measured_light) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'total_current) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'total_current_peak) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'total_power_consumed) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Status>)))
  "Returns string type for a message object of type '<Status>"
  "ridgeback_msgs/Status")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Status)))
  "Returns string type for a message object of type 'Status"
  "ridgeback_msgs/Status")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Status>)))
  "Returns md5sum for a message object of type '<Status>"
  "5b3d8e0f8c2c371cf7df823649f67044")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Status)))
  "Returns md5sum for a message object of type 'Status"
  "5b3d8e0f8c2c371cf7df823649f67044")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Status>)))
  "Returns full string definition for message of type '<Status>"
  (cl:format cl:nil "# This message represents Ridgeback's lower-frequency status updates from the MCU~%# Default publish frequency is 1Hz.~%~%Header header~%~%# Commit of firmware source.~%string hardware_id~%~%# Times since MCU power-on and MCU rosserial connection, respectively.~%duration mcu_uptime~%duration connection_uptime~%~%# Temperature of MCU's PCB in Celsius.~%float32 pcb_temperature~%float32 mcu_temperature~%~%# Monitoring the run/stop loop. Changes in these values trigger an immediate~%# publish, outside the ordinarily-scheduled 1Hz updates.~%bool stop_power_status  # True indicates the stop loop is operational (ie. it is powered).~%bool stop_engaged  # True when a stop has been pressed on the robot.~%bool drivers_active  # False when a stop is asserted to the motor drivers.~%bool external_stop_present  # True indicates a external stop has been plugged in.~%~%# Indicates if AC power is connected.~%bool charger_connected~%bool charging_complete~%~%# Voltage rails, in volts~%# Averaged over the message period~%float32 measured_battery~%float32 measured_12v~%float32 measured_5v~%float32 measured_inverter~%float32 measured_front_axle~%float32 measured_rear_axle~%float32 measured_light~%~%# Current senses available on platform, in amps.~%# Averaged over the message period~%float32 total_current~%~%# Highest total system current peak as measured in a 1ms window.~%float32 total_current_peak~%~%# Integration of all power consumption since MCU power-on, in watt-hours.~%float64 total_power_consumed~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Status)))
  "Returns full string definition for message of type 'Status"
  (cl:format cl:nil "# This message represents Ridgeback's lower-frequency status updates from the MCU~%# Default publish frequency is 1Hz.~%~%Header header~%~%# Commit of firmware source.~%string hardware_id~%~%# Times since MCU power-on and MCU rosserial connection, respectively.~%duration mcu_uptime~%duration connection_uptime~%~%# Temperature of MCU's PCB in Celsius.~%float32 pcb_temperature~%float32 mcu_temperature~%~%# Monitoring the run/stop loop. Changes in these values trigger an immediate~%# publish, outside the ordinarily-scheduled 1Hz updates.~%bool stop_power_status  # True indicates the stop loop is operational (ie. it is powered).~%bool stop_engaged  # True when a stop has been pressed on the robot.~%bool drivers_active  # False when a stop is asserted to the motor drivers.~%bool external_stop_present  # True indicates a external stop has been plugged in.~%~%# Indicates if AC power is connected.~%bool charger_connected~%bool charging_complete~%~%# Voltage rails, in volts~%# Averaged over the message period~%float32 measured_battery~%float32 measured_12v~%float32 measured_5v~%float32 measured_inverter~%float32 measured_front_axle~%float32 measured_rear_axle~%float32 measured_light~%~%# Current senses available on platform, in amps.~%# Averaged over the message period~%float32 total_current~%~%# Highest total system current peak as measured in a 1ms window.~%float32 total_current_peak~%~%# Integration of all power consumption since MCU power-on, in watt-hours.~%float64 total_power_consumed~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Status>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:length (cl:slot-value msg 'hardware_id))
     8
     8
     4
     4
     1
     1
     1
     1
     1
     1
     4
     4
     4
     4
     4
     4
     4
     4
     4
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Status>))
  "Converts a ROS message object to a list"
  (cl:list 'Status
    (cl:cons ':header (header msg))
    (cl:cons ':hardware_id (hardware_id msg))
    (cl:cons ':mcu_uptime (mcu_uptime msg))
    (cl:cons ':connection_uptime (connection_uptime msg))
    (cl:cons ':pcb_temperature (pcb_temperature msg))
    (cl:cons ':mcu_temperature (mcu_temperature msg))
    (cl:cons ':stop_power_status (stop_power_status msg))
    (cl:cons ':stop_engaged (stop_engaged msg))
    (cl:cons ':drivers_active (drivers_active msg))
    (cl:cons ':external_stop_present (external_stop_present msg))
    (cl:cons ':charger_connected (charger_connected msg))
    (cl:cons ':charging_complete (charging_complete msg))
    (cl:cons ':measured_battery (measured_battery msg))
    (cl:cons ':measured_12v (measured_12v msg))
    (cl:cons ':measured_5v (measured_5v msg))
    (cl:cons ':measured_inverter (measured_inverter msg))
    (cl:cons ':measured_front_axle (measured_front_axle msg))
    (cl:cons ':measured_rear_axle (measured_rear_axle msg))
    (cl:cons ':measured_light (measured_light msg))
    (cl:cons ':total_current (total_current msg))
    (cl:cons ':total_current_peak (total_current_peak msg))
    (cl:cons ':total_power_consumed (total_power_consumed msg))
))
