// Auto-generated. Do not edit!

// (in-package husky_gazebo_plugins.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class WheelSpeeds {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.back_left_wheel = null;
      this.back_right_wheel = null;
      this.front_left_wheel = null;
      this.front_right_wheel = null;
    }
    else {
      if (initObj.hasOwnProperty('back_left_wheel')) {
        this.back_left_wheel = initObj.back_left_wheel
      }
      else {
        this.back_left_wheel = 0.0;
      }
      if (initObj.hasOwnProperty('back_right_wheel')) {
        this.back_right_wheel = initObj.back_right_wheel
      }
      else {
        this.back_right_wheel = 0.0;
      }
      if (initObj.hasOwnProperty('front_left_wheel')) {
        this.front_left_wheel = initObj.front_left_wheel
      }
      else {
        this.front_left_wheel = 0.0;
      }
      if (initObj.hasOwnProperty('front_right_wheel')) {
        this.front_right_wheel = initObj.front_right_wheel
      }
      else {
        this.front_right_wheel = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type WheelSpeeds
    // Serialize message field [back_left_wheel]
    bufferOffset = _serializer.float64(obj.back_left_wheel, buffer, bufferOffset);
    // Serialize message field [back_right_wheel]
    bufferOffset = _serializer.float64(obj.back_right_wheel, buffer, bufferOffset);
    // Serialize message field [front_left_wheel]
    bufferOffset = _serializer.float64(obj.front_left_wheel, buffer, bufferOffset);
    // Serialize message field [front_right_wheel]
    bufferOffset = _serializer.float64(obj.front_right_wheel, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type WheelSpeeds
    let len;
    let data = new WheelSpeeds(null);
    // Deserialize message field [back_left_wheel]
    data.back_left_wheel = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [back_right_wheel]
    data.back_right_wheel = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [front_left_wheel]
    data.front_left_wheel = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [front_right_wheel]
    data.front_right_wheel = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 32;
  }

  static datatype() {
    // Returns string type for a message object
    return 'husky_gazebo_plugins/WheelSpeeds';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6ce71b24a610adb7c8e0a82f915a151f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 back_left_wheel # back left wheel speed in m/s
    float64 back_right_wheel # back right wheel speed in m/s
    float64 front_left_wheel # front left wheel speed in m/s
    float64 front_right_wheel # front right wheel speed in m/s
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new WheelSpeeds(null);
    if (msg.back_left_wheel !== undefined) {
      resolved.back_left_wheel = msg.back_left_wheel;
    }
    else {
      resolved.back_left_wheel = 0.0
    }

    if (msg.back_right_wheel !== undefined) {
      resolved.back_right_wheel = msg.back_right_wheel;
    }
    else {
      resolved.back_right_wheel = 0.0
    }

    if (msg.front_left_wheel !== undefined) {
      resolved.front_left_wheel = msg.front_left_wheel;
    }
    else {
      resolved.front_left_wheel = 0.0
    }

    if (msg.front_right_wheel !== undefined) {
      resolved.front_right_wheel = msg.front_right_wheel;
    }
    else {
      resolved.front_right_wheel = 0.0
    }

    return resolved;
    }
};

module.exports = WheelSpeeds;
