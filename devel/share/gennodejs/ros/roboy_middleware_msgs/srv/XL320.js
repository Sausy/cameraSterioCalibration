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

class XL320Request {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.type = null;
      this.motor = null;
      this.address = null;
      this.value = null;
    }
    else {
      if (initObj.hasOwnProperty('type')) {
        this.type = initObj.type
      }
      else {
        this.type = false;
      }
      if (initObj.hasOwnProperty('motor')) {
        this.motor = initObj.motor
      }
      else {
        this.motor = 0;
      }
      if (initObj.hasOwnProperty('address')) {
        this.address = initObj.address
      }
      else {
        this.address = 0;
      }
      if (initObj.hasOwnProperty('value')) {
        this.value = initObj.value
      }
      else {
        this.value = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type XL320Request
    // Serialize message field [type]
    bufferOffset = _serializer.bool(obj.type, buffer, bufferOffset);
    // Serialize message field [motor]
    bufferOffset = _serializer.uint8(obj.motor, buffer, bufferOffset);
    // Serialize message field [address]
    bufferOffset = _serializer.uint8(obj.address, buffer, bufferOffset);
    // Serialize message field [value]
    bufferOffset = _serializer.int16(obj.value, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type XL320Request
    let len;
    let data = new XL320Request(null);
    // Deserialize message field [type]
    data.type = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [motor]
    data.motor = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [address]
    data.address = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [value]
    data.value = _deserializer.int16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 5;
  }

  static datatype() {
    // Returns string type for a service object
    return 'roboy_middleware_msgs/XL320Request';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd0b3c3391dc54fe3a913cbf9ecf628c0';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    bool type
    uint8 motor
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    uint8 address
    int16 value
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new XL320Request(null);
    if (msg.type !== undefined) {
      resolved.type = msg.type;
    }
    else {
      resolved.type = false
    }

    if (msg.motor !== undefined) {
      resolved.motor = msg.motor;
    }
    else {
      resolved.motor = 0
    }

    if (msg.address !== undefined) {
      resolved.address = msg.address;
    }
    else {
      resolved.address = 0
    }

    if (msg.value !== undefined) {
      resolved.value = msg.value;
    }
    else {
      resolved.value = 0
    }

    return resolved;
    }
};

class XL320Response {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.value = null;
    }
    else {
      if (initObj.hasOwnProperty('value')) {
        this.value = initObj.value
      }
      else {
        this.value = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type XL320Response
    // Serialize message field [value]
    bufferOffset = _serializer.int16(obj.value, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type XL320Response
    let len;
    let data = new XL320Response(null);
    // Deserialize message field [value]
    data.value = _deserializer.int16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 2;
  }

  static datatype() {
    // Returns string type for a service object
    return 'roboy_middleware_msgs/XL320Response';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '55daaea9ec64e37c8a6144d42a7265e2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int16 value
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new XL320Response(null);
    if (msg.value !== undefined) {
      resolved.value = msg.value;
    }
    else {
      resolved.value = 0
    }

    return resolved;
    }
};

module.exports = {
  Request: XL320Request,
  Response: XL320Response,
  md5sum() { return '2df206e3bbdc218ed0e86d48777a9fbf'; },
  datatype() { return 'roboy_middleware_msgs/XL320'; }
};
