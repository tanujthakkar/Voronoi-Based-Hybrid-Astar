// Auto-generated. Do not edit!

// (in-package ridgeback_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class Status {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.hardware_id = null;
      this.mcu_uptime = null;
      this.connection_uptime = null;
      this.pcb_temperature = null;
      this.mcu_temperature = null;
      this.stop_power_status = null;
      this.stop_engaged = null;
      this.drivers_active = null;
      this.external_stop_present = null;
      this.charger_connected = null;
      this.charging_complete = null;
      this.measured_battery = null;
      this.measured_12v = null;
      this.measured_5v = null;
      this.measured_inverter = null;
      this.measured_front_axle = null;
      this.measured_rear_axle = null;
      this.measured_light = null;
      this.total_current = null;
      this.total_current_peak = null;
      this.total_power_consumed = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('hardware_id')) {
        this.hardware_id = initObj.hardware_id
      }
      else {
        this.hardware_id = '';
      }
      if (initObj.hasOwnProperty('mcu_uptime')) {
        this.mcu_uptime = initObj.mcu_uptime
      }
      else {
        this.mcu_uptime = {secs: 0, nsecs: 0};
      }
      if (initObj.hasOwnProperty('connection_uptime')) {
        this.connection_uptime = initObj.connection_uptime
      }
      else {
        this.connection_uptime = {secs: 0, nsecs: 0};
      }
      if (initObj.hasOwnProperty('pcb_temperature')) {
        this.pcb_temperature = initObj.pcb_temperature
      }
      else {
        this.pcb_temperature = 0.0;
      }
      if (initObj.hasOwnProperty('mcu_temperature')) {
        this.mcu_temperature = initObj.mcu_temperature
      }
      else {
        this.mcu_temperature = 0.0;
      }
      if (initObj.hasOwnProperty('stop_power_status')) {
        this.stop_power_status = initObj.stop_power_status
      }
      else {
        this.stop_power_status = false;
      }
      if (initObj.hasOwnProperty('stop_engaged')) {
        this.stop_engaged = initObj.stop_engaged
      }
      else {
        this.stop_engaged = false;
      }
      if (initObj.hasOwnProperty('drivers_active')) {
        this.drivers_active = initObj.drivers_active
      }
      else {
        this.drivers_active = false;
      }
      if (initObj.hasOwnProperty('external_stop_present')) {
        this.external_stop_present = initObj.external_stop_present
      }
      else {
        this.external_stop_present = false;
      }
      if (initObj.hasOwnProperty('charger_connected')) {
        this.charger_connected = initObj.charger_connected
      }
      else {
        this.charger_connected = false;
      }
      if (initObj.hasOwnProperty('charging_complete')) {
        this.charging_complete = initObj.charging_complete
      }
      else {
        this.charging_complete = false;
      }
      if (initObj.hasOwnProperty('measured_battery')) {
        this.measured_battery = initObj.measured_battery
      }
      else {
        this.measured_battery = 0.0;
      }
      if (initObj.hasOwnProperty('measured_12v')) {
        this.measured_12v = initObj.measured_12v
      }
      else {
        this.measured_12v = 0.0;
      }
      if (initObj.hasOwnProperty('measured_5v')) {
        this.measured_5v = initObj.measured_5v
      }
      else {
        this.measured_5v = 0.0;
      }
      if (initObj.hasOwnProperty('measured_inverter')) {
        this.measured_inverter = initObj.measured_inverter
      }
      else {
        this.measured_inverter = 0.0;
      }
      if (initObj.hasOwnProperty('measured_front_axle')) {
        this.measured_front_axle = initObj.measured_front_axle
      }
      else {
        this.measured_front_axle = 0.0;
      }
      if (initObj.hasOwnProperty('measured_rear_axle')) {
        this.measured_rear_axle = initObj.measured_rear_axle
      }
      else {
        this.measured_rear_axle = 0.0;
      }
      if (initObj.hasOwnProperty('measured_light')) {
        this.measured_light = initObj.measured_light
      }
      else {
        this.measured_light = 0.0;
      }
      if (initObj.hasOwnProperty('total_current')) {
        this.total_current = initObj.total_current
      }
      else {
        this.total_current = 0.0;
      }
      if (initObj.hasOwnProperty('total_current_peak')) {
        this.total_current_peak = initObj.total_current_peak
      }
      else {
        this.total_current_peak = 0.0;
      }
      if (initObj.hasOwnProperty('total_power_consumed')) {
        this.total_power_consumed = initObj.total_power_consumed
      }
      else {
        this.total_power_consumed = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Status
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [hardware_id]
    bufferOffset = _serializer.string(obj.hardware_id, buffer, bufferOffset);
    // Serialize message field [mcu_uptime]
    bufferOffset = _serializer.duration(obj.mcu_uptime, buffer, bufferOffset);
    // Serialize message field [connection_uptime]
    bufferOffset = _serializer.duration(obj.connection_uptime, buffer, bufferOffset);
    // Serialize message field [pcb_temperature]
    bufferOffset = _serializer.float32(obj.pcb_temperature, buffer, bufferOffset);
    // Serialize message field [mcu_temperature]
    bufferOffset = _serializer.float32(obj.mcu_temperature, buffer, bufferOffset);
    // Serialize message field [stop_power_status]
    bufferOffset = _serializer.bool(obj.stop_power_status, buffer, bufferOffset);
    // Serialize message field [stop_engaged]
    bufferOffset = _serializer.bool(obj.stop_engaged, buffer, bufferOffset);
    // Serialize message field [drivers_active]
    bufferOffset = _serializer.bool(obj.drivers_active, buffer, bufferOffset);
    // Serialize message field [external_stop_present]
    bufferOffset = _serializer.bool(obj.external_stop_present, buffer, bufferOffset);
    // Serialize message field [charger_connected]
    bufferOffset = _serializer.bool(obj.charger_connected, buffer, bufferOffset);
    // Serialize message field [charging_complete]
    bufferOffset = _serializer.bool(obj.charging_complete, buffer, bufferOffset);
    // Serialize message field [measured_battery]
    bufferOffset = _serializer.float32(obj.measured_battery, buffer, bufferOffset);
    // Serialize message field [measured_12v]
    bufferOffset = _serializer.float32(obj.measured_12v, buffer, bufferOffset);
    // Serialize message field [measured_5v]
    bufferOffset = _serializer.float32(obj.measured_5v, buffer, bufferOffset);
    // Serialize message field [measured_inverter]
    bufferOffset = _serializer.float32(obj.measured_inverter, buffer, bufferOffset);
    // Serialize message field [measured_front_axle]
    bufferOffset = _serializer.float32(obj.measured_front_axle, buffer, bufferOffset);
    // Serialize message field [measured_rear_axle]
    bufferOffset = _serializer.float32(obj.measured_rear_axle, buffer, bufferOffset);
    // Serialize message field [measured_light]
    bufferOffset = _serializer.float32(obj.measured_light, buffer, bufferOffset);
    // Serialize message field [total_current]
    bufferOffset = _serializer.float32(obj.total_current, buffer, bufferOffset);
    // Serialize message field [total_current_peak]
    bufferOffset = _serializer.float32(obj.total_current_peak, buffer, bufferOffset);
    // Serialize message field [total_power_consumed]
    bufferOffset = _serializer.float64(obj.total_power_consumed, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Status
    let len;
    let data = new Status(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [hardware_id]
    data.hardware_id = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [mcu_uptime]
    data.mcu_uptime = _deserializer.duration(buffer, bufferOffset);
    // Deserialize message field [connection_uptime]
    data.connection_uptime = _deserializer.duration(buffer, bufferOffset);
    // Deserialize message field [pcb_temperature]
    data.pcb_temperature = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [mcu_temperature]
    data.mcu_temperature = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [stop_power_status]
    data.stop_power_status = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [stop_engaged]
    data.stop_engaged = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [drivers_active]
    data.drivers_active = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [external_stop_present]
    data.external_stop_present = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [charger_connected]
    data.charger_connected = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [charging_complete]
    data.charging_complete = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [measured_battery]
    data.measured_battery = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [measured_12v]
    data.measured_12v = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [measured_5v]
    data.measured_5v = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [measured_inverter]
    data.measured_inverter = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [measured_front_axle]
    data.measured_front_axle = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [measured_rear_axle]
    data.measured_rear_axle = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [measured_light]
    data.measured_light = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [total_current]
    data.total_current = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [total_current_peak]
    data.total_current_peak = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [total_power_consumed]
    data.total_power_consumed = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += object.hardware_id.length;
    return length + 78;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ridgeback_msgs/Status';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5b3d8e0f8c2c371cf7df823649f67044';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # This message represents Ridgeback's lower-frequency status updates from the MCU
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Status(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.hardware_id !== undefined) {
      resolved.hardware_id = msg.hardware_id;
    }
    else {
      resolved.hardware_id = ''
    }

    if (msg.mcu_uptime !== undefined) {
      resolved.mcu_uptime = msg.mcu_uptime;
    }
    else {
      resolved.mcu_uptime = {secs: 0, nsecs: 0}
    }

    if (msg.connection_uptime !== undefined) {
      resolved.connection_uptime = msg.connection_uptime;
    }
    else {
      resolved.connection_uptime = {secs: 0, nsecs: 0}
    }

    if (msg.pcb_temperature !== undefined) {
      resolved.pcb_temperature = msg.pcb_temperature;
    }
    else {
      resolved.pcb_temperature = 0.0
    }

    if (msg.mcu_temperature !== undefined) {
      resolved.mcu_temperature = msg.mcu_temperature;
    }
    else {
      resolved.mcu_temperature = 0.0
    }

    if (msg.stop_power_status !== undefined) {
      resolved.stop_power_status = msg.stop_power_status;
    }
    else {
      resolved.stop_power_status = false
    }

    if (msg.stop_engaged !== undefined) {
      resolved.stop_engaged = msg.stop_engaged;
    }
    else {
      resolved.stop_engaged = false
    }

    if (msg.drivers_active !== undefined) {
      resolved.drivers_active = msg.drivers_active;
    }
    else {
      resolved.drivers_active = false
    }

    if (msg.external_stop_present !== undefined) {
      resolved.external_stop_present = msg.external_stop_present;
    }
    else {
      resolved.external_stop_present = false
    }

    if (msg.charger_connected !== undefined) {
      resolved.charger_connected = msg.charger_connected;
    }
    else {
      resolved.charger_connected = false
    }

    if (msg.charging_complete !== undefined) {
      resolved.charging_complete = msg.charging_complete;
    }
    else {
      resolved.charging_complete = false
    }

    if (msg.measured_battery !== undefined) {
      resolved.measured_battery = msg.measured_battery;
    }
    else {
      resolved.measured_battery = 0.0
    }

    if (msg.measured_12v !== undefined) {
      resolved.measured_12v = msg.measured_12v;
    }
    else {
      resolved.measured_12v = 0.0
    }

    if (msg.measured_5v !== undefined) {
      resolved.measured_5v = msg.measured_5v;
    }
    else {
      resolved.measured_5v = 0.0
    }

    if (msg.measured_inverter !== undefined) {
      resolved.measured_inverter = msg.measured_inverter;
    }
    else {
      resolved.measured_inverter = 0.0
    }

    if (msg.measured_front_axle !== undefined) {
      resolved.measured_front_axle = msg.measured_front_axle;
    }
    else {
      resolved.measured_front_axle = 0.0
    }

    if (msg.measured_rear_axle !== undefined) {
      resolved.measured_rear_axle = msg.measured_rear_axle;
    }
    else {
      resolved.measured_rear_axle = 0.0
    }

    if (msg.measured_light !== undefined) {
      resolved.measured_light = msg.measured_light;
    }
    else {
      resolved.measured_light = 0.0
    }

    if (msg.total_current !== undefined) {
      resolved.total_current = msg.total_current;
    }
    else {
      resolved.total_current = 0.0
    }

    if (msg.total_current_peak !== undefined) {
      resolved.total_current_peak = msg.total_current_peak;
    }
    else {
      resolved.total_current_peak = 0.0
    }

    if (msg.total_power_consumed !== undefined) {
      resolved.total_power_consumed = msg.total_power_consumed;
    }
    else {
      resolved.total_power_consumed = 0.0
    }

    return resolved;
    }
};

module.exports = Status;
