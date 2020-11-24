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

class DarkRoomSensorV2 {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.object_id = null;
      this.base = null;
      this.SensorID = null;
      this.elevation = null;
      this.azimuth = null;
    }
    else {
      if (initObj.hasOwnProperty('object_id')) {
        this.object_id = initObj.object_id
      }
      else {
        this.object_id = '';
      }
      if (initObj.hasOwnProperty('base')) {
        this.base = initObj.base
      }
      else {
        this.base = 0;
      }
      if (initObj.hasOwnProperty('SensorID')) {
        this.SensorID = initObj.SensorID
      }
      else {
        this.SensorID = 0;
      }
      if (initObj.hasOwnProperty('elevation')) {
        this.elevation = initObj.elevation
      }
      else {
        this.elevation = 0.0;
      }
      if (initObj.hasOwnProperty('azimuth')) {
        this.azimuth = initObj.azimuth
      }
      else {
        this.azimuth = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DarkRoomSensorV2
    // Serialize message field [object_id]
    bufferOffset = _serializer.string(obj.object_id, buffer, bufferOffset);
    // Serialize message field [base]
    bufferOffset = _serializer.int8(obj.base, buffer, bufferOffset);
    // Serialize message field [SensorID]
    bufferOffset = _serializer.int8(obj.SensorID, buffer, bufferOffset);
    // Serialize message field [elevation]
    bufferOffset = _serializer.float64(obj.elevation, buffer, bufferOffset);
    // Serialize message field [azimuth]
    bufferOffset = _serializer.float64(obj.azimuth, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DarkRoomSensorV2
    let len;
    let data = new DarkRoomSensorV2(null);
    // Deserialize message field [object_id]
    data.object_id = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [base]
    data.base = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [SensorID]
    data.SensorID = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [elevation]
    data.elevation = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [azimuth]
    data.azimuth = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.object_id.length;
    return length + 22;
  }

  static datatype() {
    // Returns string type for a message object
    return 'roboy_middleware_msgs/DarkRoomSensorV2';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '33742585e0d9d4936d60498b07ad60de';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string object_id
    int8 base
    int8 SensorID
    float64 elevation
    float64 azimuth
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new DarkRoomSensorV2(null);
    if (msg.object_id !== undefined) {
      resolved.object_id = msg.object_id;
    }
    else {
      resolved.object_id = ''
    }

    if (msg.base !== undefined) {
      resolved.base = msg.base;
    }
    else {
      resolved.base = 0
    }

    if (msg.SensorID !== undefined) {
      resolved.SensorID = msg.SensorID;
    }
    else {
      resolved.SensorID = 0
    }

    if (msg.elevation !== undefined) {
      resolved.elevation = msg.elevation;
    }
    else {
      resolved.elevation = 0.0
    }

    if (msg.azimuth !== undefined) {
      resolved.azimuth = msg.azimuth;
    }
    else {
      resolved.azimuth = 0.0
    }

    return resolved;
    }
};

module.exports = DarkRoomSensorV2;
