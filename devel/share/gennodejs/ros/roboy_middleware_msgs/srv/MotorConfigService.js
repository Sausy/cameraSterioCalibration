// Auto-generated. Do not edit!

// (in-package roboy_middleware_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let MotorConfig = require('../msg/MotorConfig.js');

//-----------------------------------------------------------


//-----------------------------------------------------------

class MotorConfigServiceRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.config = null;
    }
    else {
      if (initObj.hasOwnProperty('config')) {
        this.config = initObj.config
      }
      else {
        this.config = new MotorConfig();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MotorConfigServiceRequest
    // Serialize message field [config]
    bufferOffset = MotorConfig.serialize(obj.config, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MotorConfigServiceRequest
    let len;
    let data = new MotorConfigServiceRequest(null);
    // Deserialize message field [config]
    data.config = MotorConfig.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += MotorConfig.getMessageSize(object.config);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'roboy_middleware_msgs/MotorConfigServiceRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd32584b2ec7f004e59401de3a4cea084';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    MotorConfig config
    
    ================================================================================
    MSG: roboy_middleware_msgs/MotorConfig
    int32 id
    uint8[] motors
    uint8[] control_mode
    int32[] output_pos_max
    int32[] output_neg_max
    int32[] sp_pos_max
    int32[] sp_neg_max
    uint16[] kp
    uint16[] ki
    uint16[] kd
    uint16[] forward_gain
    uint16[] dead_band
    int32[] integral_pos_max
    int32[] integral_neg_max
    int32[] output_divider
    int32[] setpoint
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MotorConfigServiceRequest(null);
    if (msg.config !== undefined) {
      resolved.config = MotorConfig.Resolve(msg.config)
    }
    else {
      resolved.config = new MotorConfig()
    }

    return resolved;
    }
};

class MotorConfigServiceResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.mode = null;
    }
    else {
      if (initObj.hasOwnProperty('mode')) {
        this.mode = initObj.mode
      }
      else {
        this.mode = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MotorConfigServiceResponse
    // Serialize message field [mode]
    bufferOffset = _arraySerializer.int32(obj.mode, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MotorConfigServiceResponse
    let len;
    let data = new MotorConfigServiceResponse(null);
    // Deserialize message field [mode]
    data.mode = _arrayDeserializer.int32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.mode.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'roboy_middleware_msgs/MotorConfigServiceResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1fd7890fd399a8964959d3d06ecb53b5';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32[] mode
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MotorConfigServiceResponse(null);
    if (msg.mode !== undefined) {
      resolved.mode = msg.mode;
    }
    else {
      resolved.mode = []
    }

    return resolved;
    }
};

module.exports = {
  Request: MotorConfigServiceRequest,
  Response: MotorConfigServiceResponse,
  md5sum() { return '0aa4adc3b3f52f02aacbfa621182887c'; },
  datatype() { return 'roboy_middleware_msgs/MotorConfigService'; }
};
