// Auto-generated. Do not edit!

// (in-package navigation.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class img_result {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.red = null;
      this.shift = null;
    }
    else {
      if (initObj.hasOwnProperty('red')) {
        this.red = initObj.red
      }
      else {
        this.red = '';
      }
      if (initObj.hasOwnProperty('shift')) {
        this.shift = initObj.shift
      }
      else {
        this.shift = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type img_result
    // Serialize message field [red]
    bufferOffset = _serializer.string(obj.red, buffer, bufferOffset);
    // Serialize message field [shift]
    bufferOffset = _serializer.int16(obj.shift, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type img_result
    let len;
    let data = new img_result(null);
    // Deserialize message field [red]
    data.red = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [shift]
    data.shift = _deserializer.int16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.red);
    return length + 6;
  }

  static datatype() {
    // Returns string type for a message object
    return 'navigation/img_result';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd21b5d7806ece2676a2ff5c007553324';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string red 
    int16 shift
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new img_result(null);
    if (msg.red !== undefined) {
      resolved.red = msg.red;
    }
    else {
      resolved.red = ''
    }

    if (msg.shift !== undefined) {
      resolved.shift = msg.shift;
    }
    else {
      resolved.shift = 0
    }

    return resolved;
    }
};

module.exports = img_result;
