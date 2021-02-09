;; Auto-generated. Do not edit!


(when (boundp 'ridgeback_msgs::Status)
  (if (not (find-package "RIDGEBACK_MSGS"))
    (make-package "RIDGEBACK_MSGS"))
  (shadow 'Status (find-package "RIDGEBACK_MSGS")))
(unless (find-package "RIDGEBACK_MSGS::STATUS")
  (make-package "RIDGEBACK_MSGS::STATUS"))

(in-package "ROS")
;;//! \htmlinclude Status.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass ridgeback_msgs::Status
  :super ros::object
  :slots (_header _hardware_id _mcu_uptime _connection_uptime _pcb_temperature _mcu_temperature _stop_power_status _stop_engaged _drivers_active _external_stop_present _charger_connected _charging_complete _measured_battery _measured_12v _measured_5v _measured_inverter _measured_front_axle _measured_rear_axle _measured_light _total_current _total_current_peak _total_power_consumed ))

(defmethod ridgeback_msgs::Status
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:hardware_id __hardware_id) "")
    ((:mcu_uptime __mcu_uptime) (instance ros::time :init))
    ((:connection_uptime __connection_uptime) (instance ros::time :init))
    ((:pcb_temperature __pcb_temperature) 0.0)
    ((:mcu_temperature __mcu_temperature) 0.0)
    ((:stop_power_status __stop_power_status) nil)
    ((:stop_engaged __stop_engaged) nil)
    ((:drivers_active __drivers_active) nil)
    ((:external_stop_present __external_stop_present) nil)
    ((:charger_connected __charger_connected) nil)
    ((:charging_complete __charging_complete) nil)
    ((:measured_battery __measured_battery) 0.0)
    ((:measured_12v __measured_12v) 0.0)
    ((:measured_5v __measured_5v) 0.0)
    ((:measured_inverter __measured_inverter) 0.0)
    ((:measured_front_axle __measured_front_axle) 0.0)
    ((:measured_rear_axle __measured_rear_axle) 0.0)
    ((:measured_light __measured_light) 0.0)
    ((:total_current __total_current) 0.0)
    ((:total_current_peak __total_current_peak) 0.0)
    ((:total_power_consumed __total_power_consumed) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _hardware_id (string __hardware_id))
   (setq _mcu_uptime __mcu_uptime)
   (setq _connection_uptime __connection_uptime)
   (setq _pcb_temperature (float __pcb_temperature))
   (setq _mcu_temperature (float __mcu_temperature))
   (setq _stop_power_status __stop_power_status)
   (setq _stop_engaged __stop_engaged)
   (setq _drivers_active __drivers_active)
   (setq _external_stop_present __external_stop_present)
   (setq _charger_connected __charger_connected)
   (setq _charging_complete __charging_complete)
   (setq _measured_battery (float __measured_battery))
   (setq _measured_12v (float __measured_12v))
   (setq _measured_5v (float __measured_5v))
   (setq _measured_inverter (float __measured_inverter))
   (setq _measured_front_axle (float __measured_front_axle))
   (setq _measured_rear_axle (float __measured_rear_axle))
   (setq _measured_light (float __measured_light))
   (setq _total_current (float __total_current))
   (setq _total_current_peak (float __total_current_peak))
   (setq _total_power_consumed (float __total_power_consumed))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:hardware_id
   (&optional __hardware_id)
   (if __hardware_id (setq _hardware_id __hardware_id)) _hardware_id)
  (:mcu_uptime
   (&optional __mcu_uptime)
   (if __mcu_uptime (setq _mcu_uptime __mcu_uptime)) _mcu_uptime)
  (:connection_uptime
   (&optional __connection_uptime)
   (if __connection_uptime (setq _connection_uptime __connection_uptime)) _connection_uptime)
  (:pcb_temperature
   (&optional __pcb_temperature)
   (if __pcb_temperature (setq _pcb_temperature __pcb_temperature)) _pcb_temperature)
  (:mcu_temperature
   (&optional __mcu_temperature)
   (if __mcu_temperature (setq _mcu_temperature __mcu_temperature)) _mcu_temperature)
  (:stop_power_status
   (&optional __stop_power_status)
   (if __stop_power_status (setq _stop_power_status __stop_power_status)) _stop_power_status)
  (:stop_engaged
   (&optional __stop_engaged)
   (if __stop_engaged (setq _stop_engaged __stop_engaged)) _stop_engaged)
  (:drivers_active
   (&optional __drivers_active)
   (if __drivers_active (setq _drivers_active __drivers_active)) _drivers_active)
  (:external_stop_present
   (&optional __external_stop_present)
   (if __external_stop_present (setq _external_stop_present __external_stop_present)) _external_stop_present)
  (:charger_connected
   (&optional __charger_connected)
   (if __charger_connected (setq _charger_connected __charger_connected)) _charger_connected)
  (:charging_complete
   (&optional __charging_complete)
   (if __charging_complete (setq _charging_complete __charging_complete)) _charging_complete)
  (:measured_battery
   (&optional __measured_battery)
   (if __measured_battery (setq _measured_battery __measured_battery)) _measured_battery)
  (:measured_12v
   (&optional __measured_12v)
   (if __measured_12v (setq _measured_12v __measured_12v)) _measured_12v)
  (:measured_5v
   (&optional __measured_5v)
   (if __measured_5v (setq _measured_5v __measured_5v)) _measured_5v)
  (:measured_inverter
   (&optional __measured_inverter)
   (if __measured_inverter (setq _measured_inverter __measured_inverter)) _measured_inverter)
  (:measured_front_axle
   (&optional __measured_front_axle)
   (if __measured_front_axle (setq _measured_front_axle __measured_front_axle)) _measured_front_axle)
  (:measured_rear_axle
   (&optional __measured_rear_axle)
   (if __measured_rear_axle (setq _measured_rear_axle __measured_rear_axle)) _measured_rear_axle)
  (:measured_light
   (&optional __measured_light)
   (if __measured_light (setq _measured_light __measured_light)) _measured_light)
  (:total_current
   (&optional __total_current)
   (if __total_current (setq _total_current __total_current)) _total_current)
  (:total_current_peak
   (&optional __total_current_peak)
   (if __total_current_peak (setq _total_current_peak __total_current_peak)) _total_current_peak)
  (:total_power_consumed
   (&optional __total_power_consumed)
   (if __total_power_consumed (setq _total_power_consumed __total_power_consumed)) _total_power_consumed)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _hardware_id
    4 (length _hardware_id)
    ;; duration _mcu_uptime
    8
    ;; duration _connection_uptime
    8
    ;; float32 _pcb_temperature
    4
    ;; float32 _mcu_temperature
    4
    ;; bool _stop_power_status
    1
    ;; bool _stop_engaged
    1
    ;; bool _drivers_active
    1
    ;; bool _external_stop_present
    1
    ;; bool _charger_connected
    1
    ;; bool _charging_complete
    1
    ;; float32 _measured_battery
    4
    ;; float32 _measured_12v
    4
    ;; float32 _measured_5v
    4
    ;; float32 _measured_inverter
    4
    ;; float32 _measured_front_axle
    4
    ;; float32 _measured_rear_axle
    4
    ;; float32 _measured_light
    4
    ;; float32 _total_current
    4
    ;; float32 _total_current_peak
    4
    ;; float64 _total_power_consumed
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _hardware_id
       (write-long (length _hardware_id) s) (princ _hardware_id s)
     ;; duration _mcu_uptime
       (write-long (send _mcu_uptime :sec) s) (write-long (send _mcu_uptime :nsec) s)
     ;; duration _connection_uptime
       (write-long (send _connection_uptime :sec) s) (write-long (send _connection_uptime :nsec) s)
     ;; float32 _pcb_temperature
       (sys::poke _pcb_temperature (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _mcu_temperature
       (sys::poke _mcu_temperature (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool _stop_power_status
       (if _stop_power_status (write-byte -1 s) (write-byte 0 s))
     ;; bool _stop_engaged
       (if _stop_engaged (write-byte -1 s) (write-byte 0 s))
     ;; bool _drivers_active
       (if _drivers_active (write-byte -1 s) (write-byte 0 s))
     ;; bool _external_stop_present
       (if _external_stop_present (write-byte -1 s) (write-byte 0 s))
     ;; bool _charger_connected
       (if _charger_connected (write-byte -1 s) (write-byte 0 s))
     ;; bool _charging_complete
       (if _charging_complete (write-byte -1 s) (write-byte 0 s))
     ;; float32 _measured_battery
       (sys::poke _measured_battery (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _measured_12v
       (sys::poke _measured_12v (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _measured_5v
       (sys::poke _measured_5v (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _measured_inverter
       (sys::poke _measured_inverter (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _measured_front_axle
       (sys::poke _measured_front_axle (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _measured_rear_axle
       (sys::poke _measured_rear_axle (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _measured_light
       (sys::poke _measured_light (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _total_current
       (sys::poke _total_current (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _total_current_peak
       (sys::poke _total_current_peak (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float64 _total_power_consumed
       (sys::poke _total_power_consumed (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _hardware_id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _hardware_id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; duration _mcu_uptime
     (send _mcu_uptime :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _mcu_uptime :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; duration _connection_uptime
     (send _connection_uptime :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _connection_uptime :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _pcb_temperature
     (setq _pcb_temperature (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _mcu_temperature
     (setq _mcu_temperature (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _stop_power_status
     (setq _stop_power_status (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _stop_engaged
     (setq _stop_engaged (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _drivers_active
     (setq _drivers_active (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _external_stop_present
     (setq _external_stop_present (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _charger_connected
     (setq _charger_connected (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _charging_complete
     (setq _charging_complete (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float32 _measured_battery
     (setq _measured_battery (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _measured_12v
     (setq _measured_12v (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _measured_5v
     (setq _measured_5v (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _measured_inverter
     (setq _measured_inverter (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _measured_front_axle
     (setq _measured_front_axle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _measured_rear_axle
     (setq _measured_rear_axle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _measured_light
     (setq _measured_light (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _total_current
     (setq _total_current (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _total_current_peak
     (setq _total_current_peak (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float64 _total_power_consumed
     (setq _total_power_consumed (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get ridgeback_msgs::Status :md5sum-) "5b3d8e0f8c2c371cf7df823649f67044")
(setf (get ridgeback_msgs::Status :datatype-) "ridgeback_msgs/Status")
(setf (get ridgeback_msgs::Status :definition-)
      "# This message represents Ridgeback's lower-frequency status updates from the MCU
# Default publish frequency is 1Hz.

Header header

# Commit of firmware source.
string hardware_id

# Times since MCU power-on and MCU rosserial connection, respectively.
duration mcu_uptime
duration connection_uptime

# Temperature of MCU's PCB in Celsius.
float32 pcb_temperature
float32 mcu_temperature

# Monitoring the run/stop loop. Changes in these values trigger an immediate
# publish, outside the ordinarily-scheduled 1Hz updates.
bool stop_power_status  # True indicates the stop loop is operational (ie. it is powered).
bool stop_engaged  # True when a stop has been pressed on the robot.
bool drivers_active  # False when a stop is asserted to the motor drivers.
bool external_stop_present  # True indicates a external stop has been plugged in.

# Indicates if AC power is connected.
bool charger_connected
bool charging_complete

# Voltage rails, in volts
# Averaged over the message period
float32 measured_battery
float32 measured_12v
float32 measured_5v
float32 measured_inverter
float32 measured_front_axle
float32 measured_rear_axle
float32 measured_light

# Current senses available on platform, in amps.
# Averaged over the message period
float32 total_current

# Highest total system current peak as measured in a 1ms window.
float32 total_current_peak

# Integration of all power consumption since MCU power-on, in watt-hours.
float64 total_power_consumed

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

")



(provide :ridgeback_msgs/Status "5b3d8e0f8c2c371cf7df823649f67044")


