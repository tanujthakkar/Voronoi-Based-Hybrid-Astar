// Auto-generated. Do not edit!

// (in-package ridgeback_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let RGB = require('./RGB.js');

//-----------------------------------------------------------

class Lights {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.lights = null;
    }
    else {
      if (initObj.hasOwnProperty('lights')) {
        this.lights = initObj.lights
      }
      else {
        this.lights = new Array(8).fill(new RGB());
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Lights
    // Check that the constant length array field [lights] has the right length
    if (obj.lights.length !== 8) {
      throw new Error('Unable to serialize array field lights - length must be 8')
    }
    // Serialize message field [lights]
    obj.lights.forEach((val) => {
      bufferOffset = RGB.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Lights
    let len;
    let data = new Lights(null);
    // Deserialize message field [lights]
    len = 8;
    data.lights = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.lights[i] = RGB.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    return 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ridgeback_msgs/Lights';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2c68505ba4cf8e160d2760ed01777bc7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Represents a command for the 8 RGB body lights on Ridgeback.
    
    uint8 LIGHTS_FRONT_LEFT_UPPER=0
    uint8 LIGHTS_FRONT_LEFT_LOWER=1
    uint8 LIGHTS_FRONT_RIGHT_UPPER=2
    uint8 LIGHTS_FRONT_RIGHT_LOWER=3
    uint8 LIGHTS_REAR_LEFT_UPPER=4
    uint8 LIGHTS_REAR_LEFT_LOWER=5
    uint8 LIGHTS_REAR_RIGHT_UPPER=6
    uint8 LIGHTS_REAR_RIGHT_LOWER=7
    
    ridgeback_msgs/RGB[8] lights
    
    ================================================================================
    MSG: ridgeback_msgs/RGB
    # Represents the intensity of a single RGB LED, either reported or commanded.
    
    float32 red
    float32 green
    float32 blue
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Lights(null);
    if (msg.lights !== undefined) {
      resolved.lights = new Array(8)
      for (let i = 0; i < resolved.lights.length; ++i) {
        if (msg.lights.length > i) {
          resolved.lights[i] = RGB.Resolve(msg.lights[i]);
        }
        else {
          resolved.lights[i] = new RGB();
        }
      }
    }
    else {
      resolved.lights = new Array(8).fill(new RGB())
    }

    return resolved;
    }
};

// Constants for message
Lights.Constants = {
  LIGHTS_FRONT_LEFT_UPPER: 0,
  LIGHTS_FRONT_LEFT_LOWER: 1,
  LIGHTS_FRONT_RIGHT_UPPER: 2,
  LIGHTS_FRONT_RIGHT_LOWER: 3,
  LIGHTS_REAR_LEFT_UPPER: 4,
  LIGHTS_REAR_LEFT_LOWER: 5,
  LIGHTS_REAR_RIGHT_UPPER: 6,
  LIGHTS_REAR_RIGHT_LOWER: 7,
}

module.exports = Lights;
