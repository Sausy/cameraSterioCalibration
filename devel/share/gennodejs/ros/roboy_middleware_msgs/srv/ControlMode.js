// Auto-generated. Do not edit!

// (in-package roboy_middleware_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class ControlModeRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.control_mode = null;
      this.set_point = null;
      this.motor_id = null;
    }
    else {
      if (initObj.hasOwnProperty('control_mode')) {
        this.control_mode = initObj.control_mode
      }
      else {
        this.control_mode = 0;
      }
      if (initObj.hasOwnProperty('set_point')) {
        this.set_point = initObj.set_point
      }
      else {
        this.set_point = 0;
      }
      if (initObj.hasOwnProperty('motor_id')) {
        this.motor_id = initObj.motor_id
      }
      else {
        this.motor_id = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ControlModeRequest
    // Serialize message field [control_mode]
    bufferOffset = _serializer.int32(obj.control_mode, buffer, bufferOffset);
    // Serialize message field [set_point]
    bufferOffset = _serializer.int32(obj.set_point, buffer, bufferOffset);
    // Serialize message field [motor_id]
    bufferOffset = _arraySerializer.uint16(obj.motor_id, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ControlModeRequest
    let len;
    let data = new ControlModeRequest(null);
    // Deserialize message field [control_mode]
    data.control_mode = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [set_point]
    data.set_point = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [motor_id]
    data.motor_id = _arrayDeserializer.uint16(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 2 * object.motor_id.length;
    return length + 12;
  }

  static datatype() {
    // Returns string type for a service object
    return 'roboy_middleware_msgs/ControlModeRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3bf3f3c500bf7df65355716be531bee1';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 control_mode
    int32 set_point
    
    uint16[] motor_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ControlModeRequest(null);
    if (msg.control_mode !== undefined) {
      resolved.control_mode = msg.control_mode;
    }
    else {
      resolved.control_mode = 0
    }

    if (msg.set_point !== undefined) {
      resolved.set_point = msg.set_point;
    }
    else {
      resolved.set_point = 0
    }

    if (msg.motor_id !== undefined) {
      resolved.motor_id = msg.motor_id;
    }
    else {
      resolved.motor_id = []
    }

    return resolved;
    }
};

class ControlModeResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ControlModeResponse
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ControlModeResponse
    let len;
    let data = new ControlModeResponse(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'roboy_middleware_msgs/ControlModeResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ControlModeResponse(null);
    return resolved;
    }
};

module.exports = {
  Request: ControlModeRequest,
  Response: ControlModeResponse,
  md5sum() { return '3bf3f3c500bf7df65355716be531bee1'; },
  datatype() { return 'roboy_middleware_msgs/ControlMode'; }
};
