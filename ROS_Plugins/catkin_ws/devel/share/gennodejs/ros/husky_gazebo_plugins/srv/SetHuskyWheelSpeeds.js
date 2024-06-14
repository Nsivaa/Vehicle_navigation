// Auto-generated. Do not edit!

// (in-package husky_gazebo_plugins.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let WheelSpeeds = require('../msg/WheelSpeeds.js');

//-----------------------------------------------------------


//-----------------------------------------------------------

class SetHuskyWheelSpeedsRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.wheel_speeds = null;
    }
    else {
      if (initObj.hasOwnProperty('wheel_speeds')) {
        this.wheel_speeds = initObj.wheel_speeds
      }
      else {
        this.wheel_speeds = new WheelSpeeds();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetHuskyWheelSpeedsRequest
    // Serialize message field [wheel_speeds]
    bufferOffset = WheelSpeeds.serialize(obj.wheel_speeds, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetHuskyWheelSpeedsRequest
    let len;
    let data = new SetHuskyWheelSpeedsRequest(null);
    // Deserialize message field [wheel_speeds]
    data.wheel_speeds = WheelSpeeds.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 32;
  }

  static datatype() {
    // Returns string type for a service object
    return 'husky_gazebo_plugins/SetHuskyWheelSpeedsRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ea93786adaebbf7a0cdc458efcc81166';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    husky_gazebo_plugins/WheelSpeeds wheel_speeds
    
    ================================================================================
    MSG: husky_gazebo_plugins/WheelSpeeds
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
    const resolved = new SetHuskyWheelSpeedsRequest(null);
    if (msg.wheel_speeds !== undefined) {
      resolved.wheel_speeds = WheelSpeeds.Resolve(msg.wheel_speeds)
    }
    else {
      resolved.wheel_speeds = new WheelSpeeds()
    }

    return resolved;
    }
};

class SetHuskyWheelSpeedsResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
      this.status_message = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
      if (initObj.hasOwnProperty('status_message')) {
        this.status_message = initObj.status_message
      }
      else {
        this.status_message = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetHuskyWheelSpeedsResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [status_message]
    bufferOffset = _serializer.string(obj.status_message, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetHuskyWheelSpeedsResponse
    let len;
    let data = new SetHuskyWheelSpeedsResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [status_message]
    data.status_message = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.status_message);
    return length + 5;
  }

  static datatype() {
    // Returns string type for a service object
    return 'husky_gazebo_plugins/SetHuskyWheelSpeedsResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2ec6f3eff0161f4257b808b12bc830c2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool success                         # return true if set successful
    string status_message                # comments if available
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetHuskyWheelSpeedsResponse(null);
    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    if (msg.status_message !== undefined) {
      resolved.status_message = msg.status_message;
    }
    else {
      resolved.status_message = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: SetHuskyWheelSpeedsRequest,
  Response: SetHuskyWheelSpeedsResponse,
  md5sum() { return 'ebe9a100c73aa02179b4e91ad134a89b'; },
  datatype() { return 'husky_gazebo_plugins/SetHuskyWheelSpeeds'; }
};
