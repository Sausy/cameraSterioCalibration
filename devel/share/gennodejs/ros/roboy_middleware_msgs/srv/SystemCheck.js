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

class SystemCheckRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.motorids = null;
    }
    else {
      if (initObj.hasOwnProperty('motorids')) {
        this.motorids = initObj.motorids
      }
      else {
        this.motorids = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SystemCheckRequest
    // Serialize message field [motorids]
    bufferOffset = _arraySerializer.uint8(obj.motorids, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SystemCheckRequest
    let len;
    let data = new SystemCheckRequest(null);
    // Deserialize message field [motorids]
    data.motorids = _arrayDeserializer.uint8(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.motorids.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'roboy_middleware_msgs/SystemCheckRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a6e886e25b6e52eefb773ca5d68c5bb7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint8[] motorids
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SystemCheckRequest(null);
    if (msg.motorids !== undefined) {
      resolved.motorids = msg.motorids;
    }
    else {
      resolved.motorids = []
    }

    return resolved;
    }
};

class SystemCheckResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.position = null;
      this.displacement = null;
    }
    else {
      if (initObj.hasOwnProperty('position')) {
        this.position = initObj.position
      }
      else {
        this.position = [];
      }
      if (initObj.hasOwnProperty('displacement')) {
        this.displacement = initObj.displacement
      }
      else {
        this.displacement = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SystemCheckResponse
    // Serialize message field [position]
    bufferOffset = _arraySerializer.bool(obj.position, buffer, bufferOffset, null);
    // Serialize message field [displacement]
    bufferOffset = _arraySerializer.bool(obj.displacement, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SystemCheckResponse
    let len;
    let data = new SystemCheckResponse(null);
    // Deserialize message field [position]
    data.position = _arrayDeserializer.bool(buffer, bufferOffset, null)
    // Deserialize message field [displacement]
    data.displacement = _arrayDeserializer.bool(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.position.length;
    length += object.displacement.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'roboy_middleware_msgs/SystemCheckResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4b2adf9f4e24ef5dc53bf57a7793ebcb';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    bool[] position
    bool[] displacement
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SystemCheckResponse(null);
    if (msg.position !== undefined) {
      resolved.position = msg.position;
    }
    else {
      resolved.position = []
    }

    if (msg.displacement !== undefined) {
      resolved.displacement = msg.displacement;
    }
    else {
      resolved.displacement = []
    }

    return resolved;
    }
};

module.exports = {
  Request: SystemCheckRequest,
  Response: SystemCheckResponse,
  md5sum() { return 'd2bdc9a40e1feeae452cdda757d15ee0'; },
  datatype() { return 'roboy_middleware_msgs/SystemCheck'; }
};
