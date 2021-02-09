// Auto-generated. Do not edit!

// (in-package ridgeback_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Fans {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.fans = null;
    }
    else {
      if (initObj.hasOwnProperty('fans')) {
        this.fans = initObj.fans
      }
      else {
        this.fans = new Array(6).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Fans
    // Check that the constant length array field [fans] has the right length
    if (obj.fans.length !== 6) {
      throw new Error('Unable to serialize array field fans - length must be 6')
    }
    // Serialize message field [fans]
    bufferOffset = _arraySerializer.uint8(obj.fans, buffer, bufferOffset, 6);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Fans
    let len;
    let data = new Fans(null);
    // Deserialize message field [fans]
    data.fans = _arrayDeserializer.uint8(buffer, bufferOffset, 6)
    return data;
  }

  static getMessageSize(object) {
    return 6;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ridgeback_msgs/Fans';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd529aec610975f8df12d912730064bbf';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Location of fans.
    uint8 EQUIPMENT_BAY_INTAKE=0
    uint8 EQUIPMENT_BAY_EXHAUST=1
    uint8 CHARGER_BAY_INTAKE=2
    uint8 CHARGER_BAY_EXHAUST=3
    uint8 USER_BAY_INTAKE=4
    uint8 USER_BAY_EXHAUST=5
    
    uint8 FAN_OFF=0
    uint8 FAN_ON_HIGH=1
    uint8 FAN_ON_LOW=2
    uint8[6] fans
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Fans(null);
    if (msg.fans !== undefined) {
      resolved.fans = msg.fans;
    }
    else {
      resolved.fans = new Array(6).fill(0)
    }

    return resolved;
    }
};

// Constants for message
Fans.Constants = {
  EQUIPMENT_BAY_INTAKE: 0,
  EQUIPMENT_BAY_EXHAUST: 1,
  CHARGER_BAY_INTAKE: 2,
  CHARGER_BAY_EXHAUST: 3,
  USER_BAY_INTAKE: 4,
  USER_BAY_EXHAUST: 5,
  FAN_OFF: 0,
  FAN_ON_HIGH: 1,
  FAN_ON_LOW: 2,
}

module.exports = Fans;
