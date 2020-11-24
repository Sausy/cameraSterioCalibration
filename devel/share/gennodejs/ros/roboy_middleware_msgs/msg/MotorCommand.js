// Auto-generated. Do not edit!

// (in-package roboy_middleware_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class MotorCommand {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.id = null;
      this.motors = null;
      this.set_points = null;
    }
    else {
      if (initObj.hasOwnProperty('id')) {
        this.id = initObj.id
      }
      else {
        this.id = 0;
      }
      if (initObj.hasOwnProperty('motors')) {
        this.motors = initObj.motors
      }
      else {
        this.motors = [];
      }
      if (initObj.hasOwnProperty('set_points')) {
        this.set_points = initObj.set_points
      }
      else {
        this.set_points = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MotorCommand
    // Serialize message field [id]
    bufferOffset = _serializer.uint8(obj.id, buffer, bufferOffset);
    // Serialize message field [motors]
    bufferOffset = _arraySerializer.int32(obj.motors, buffer, bufferOffset, null);
    // Serialize message field [set_points]
    bufferOffset = _arraySerializer.float32(obj.set_points, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MotorCommand
    let len;
    let data = new MotorCommand(null);
    // Deserialize message field [id]
    data.id = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [motors]
    data.motors = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [set_points]
    data.set_points = _arrayDeserializer.float32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.motors.length;
    length += 4 * object.set_points.length;
    return length + 9;
  }

  static datatype() {
    // Returns string type for a message object
    return 'roboy_middleware_msgs/MotorCommand';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '39efb397cd93ab4cb00018c2fb0a7c3c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8 id
    int32[] motors
    float32[] set_points
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MotorCommand(null);
    if (msg.id !== undefined) {
      resolved.id = msg.id;
    }
    else {
      resolved.id = 0
    }

    if (msg.motors !== undefined) {
      resolved.motors = msg.motors;
    }
    else {
      resolved.motors = []
    }

    if (msg.set_points !== undefined) {
      resolved.set_points = msg.set_points;
    }
    else {
      resolved.set_points = []
    }

    return resolved;
    }
};

module.exports = MotorCommand;
