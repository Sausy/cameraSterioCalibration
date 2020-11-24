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

class JointControllerRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.elbow_control_enable = null;
      this.kp_elbow_joint = null;
      this.kd_elbow_joint = null;
      this.kp_elbow_agonist = null;
      this.kd_elbow_agonist = null;
      this.kp_elbow_antagonist = null;
      this.kd_elbow_antagonist = null;
      this.elbow_deadband = null;
      this.elbow_pretension = null;
      this.wrist_control_enable = null;
      this.kp_wrist_joint = null;
      this.kd_wrist_joint = null;
      this.kp_wrist_agonist = null;
      this.kd_wrist_agonist = null;
      this.kp_wrist_antagonist = null;
      this.kd_wrist_antagonist = null;
      this.wrist_deadband = null;
      this.wrist_pretension = null;
    }
    else {
      if (initObj.hasOwnProperty('elbow_control_enable')) {
        this.elbow_control_enable = initObj.elbow_control_enable
      }
      else {
        this.elbow_control_enable = false;
      }
      if (initObj.hasOwnProperty('kp_elbow_joint')) {
        this.kp_elbow_joint = initObj.kp_elbow_joint
      }
      else {
        this.kp_elbow_joint = 0;
      }
      if (initObj.hasOwnProperty('kd_elbow_joint')) {
        this.kd_elbow_joint = initObj.kd_elbow_joint
      }
      else {
        this.kd_elbow_joint = 0;
      }
      if (initObj.hasOwnProperty('kp_elbow_agonist')) {
        this.kp_elbow_agonist = initObj.kp_elbow_agonist
      }
      else {
        this.kp_elbow_agonist = 0;
      }
      if (initObj.hasOwnProperty('kd_elbow_agonist')) {
        this.kd_elbow_agonist = initObj.kd_elbow_agonist
      }
      else {
        this.kd_elbow_agonist = 0;
      }
      if (initObj.hasOwnProperty('kp_elbow_antagonist')) {
        this.kp_elbow_antagonist = initObj.kp_elbow_antagonist
      }
      else {
        this.kp_elbow_antagonist = 0;
      }
      if (initObj.hasOwnProperty('kd_elbow_antagonist')) {
        this.kd_elbow_antagonist = initObj.kd_elbow_antagonist
      }
      else {
        this.kd_elbow_antagonist = 0;
      }
      if (initObj.hasOwnProperty('elbow_deadband')) {
        this.elbow_deadband = initObj.elbow_deadband
      }
      else {
        this.elbow_deadband = 0;
      }
      if (initObj.hasOwnProperty('elbow_pretension')) {
        this.elbow_pretension = initObj.elbow_pretension
      }
      else {
        this.elbow_pretension = 0;
      }
      if (initObj.hasOwnProperty('wrist_control_enable')) {
        this.wrist_control_enable = initObj.wrist_control_enable
      }
      else {
        this.wrist_control_enable = false;
      }
      if (initObj.hasOwnProperty('kp_wrist_joint')) {
        this.kp_wrist_joint = initObj.kp_wrist_joint
      }
      else {
        this.kp_wrist_joint = 0;
      }
      if (initObj.hasOwnProperty('kd_wrist_joint')) {
        this.kd_wrist_joint = initObj.kd_wrist_joint
      }
      else {
        this.kd_wrist_joint = 0;
      }
      if (initObj.hasOwnProperty('kp_wrist_agonist')) {
        this.kp_wrist_agonist = initObj.kp_wrist_agonist
      }
      else {
        this.kp_wrist_agonist = 0;
      }
      if (initObj.hasOwnProperty('kd_wrist_agonist')) {
        this.kd_wrist_agonist = initObj.kd_wrist_agonist
      }
      else {
        this.kd_wrist_agonist = 0;
      }
      if (initObj.hasOwnProperty('kp_wrist_antagonist')) {
        this.kp_wrist_antagonist = initObj.kp_wrist_antagonist
      }
      else {
        this.kp_wrist_antagonist = 0;
      }
      if (initObj.hasOwnProperty('kd_wrist_antagonist')) {
        this.kd_wrist_antagonist = initObj.kd_wrist_antagonist
      }
      else {
        this.kd_wrist_antagonist = 0;
      }
      if (initObj.hasOwnProperty('wrist_deadband')) {
        this.wrist_deadband = initObj.wrist_deadband
      }
      else {
        this.wrist_deadband = 0;
      }
      if (initObj.hasOwnProperty('wrist_pretension')) {
        this.wrist_pretension = initObj.wrist_pretension
      }
      else {
        this.wrist_pretension = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type JointControllerRequest
    // Serialize message field [elbow_control_enable]
    bufferOffset = _serializer.bool(obj.elbow_control_enable, buffer, bufferOffset);
    // Serialize message field [kp_elbow_joint]
    bufferOffset = _serializer.int32(obj.kp_elbow_joint, buffer, bufferOffset);
    // Serialize message field [kd_elbow_joint]
    bufferOffset = _serializer.int32(obj.kd_elbow_joint, buffer, bufferOffset);
    // Serialize message field [kp_elbow_agonist]
    bufferOffset = _serializer.int32(obj.kp_elbow_agonist, buffer, bufferOffset);
    // Serialize message field [kd_elbow_agonist]
    bufferOffset = _serializer.int32(obj.kd_elbow_agonist, buffer, bufferOffset);
    // Serialize message field [kp_elbow_antagonist]
    bufferOffset = _serializer.int32(obj.kp_elbow_antagonist, buffer, bufferOffset);
    // Serialize message field [kd_elbow_antagonist]
    bufferOffset = _serializer.int32(obj.kd_elbow_antagonist, buffer, bufferOffset);
    // Serialize message field [elbow_deadband]
    bufferOffset = _serializer.int32(obj.elbow_deadband, buffer, bufferOffset);
    // Serialize message field [elbow_pretension]
    bufferOffset = _serializer.int32(obj.elbow_pretension, buffer, bufferOffset);
    // Serialize message field [wrist_control_enable]
    bufferOffset = _serializer.bool(obj.wrist_control_enable, buffer, bufferOffset);
    // Serialize message field [kp_wrist_joint]
    bufferOffset = _serializer.int32(obj.kp_wrist_joint, buffer, bufferOffset);
    // Serialize message field [kd_wrist_joint]
    bufferOffset = _serializer.int32(obj.kd_wrist_joint, buffer, bufferOffset);
    // Serialize message field [kp_wrist_agonist]
    bufferOffset = _serializer.int32(obj.kp_wrist_agonist, buffer, bufferOffset);
    // Serialize message field [kd_wrist_agonist]
    bufferOffset = _serializer.int32(obj.kd_wrist_agonist, buffer, bufferOffset);
    // Serialize message field [kp_wrist_antagonist]
    bufferOffset = _serializer.int32(obj.kp_wrist_antagonist, buffer, bufferOffset);
    // Serialize message field [kd_wrist_antagonist]
    bufferOffset = _serializer.int32(obj.kd_wrist_antagonist, buffer, bufferOffset);
    // Serialize message field [wrist_deadband]
    bufferOffset = _serializer.int32(obj.wrist_deadband, buffer, bufferOffset);
    // Serialize message field [wrist_pretension]
    bufferOffset = _serializer.int32(obj.wrist_pretension, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type JointControllerRequest
    let len;
    let data = new JointControllerRequest(null);
    // Deserialize message field [elbow_control_enable]
    data.elbow_control_enable = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [kp_elbow_joint]
    data.kp_elbow_joint = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [kd_elbow_joint]
    data.kd_elbow_joint = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [kp_elbow_agonist]
    data.kp_elbow_agonist = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [kd_elbow_agonist]
    data.kd_elbow_agonist = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [kp_elbow_antagonist]
    data.kp_elbow_antagonist = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [kd_elbow_antagonist]
    data.kd_elbow_antagonist = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [elbow_deadband]
    data.elbow_deadband = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [elbow_pretension]
    data.elbow_pretension = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [wrist_control_enable]
    data.wrist_control_enable = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [kp_wrist_joint]
    data.kp_wrist_joint = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [kd_wrist_joint]
    data.kd_wrist_joint = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [kp_wrist_agonist]
    data.kp_wrist_agonist = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [kd_wrist_agonist]
    data.kd_wrist_agonist = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [kp_wrist_antagonist]
    data.kp_wrist_antagonist = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [kd_wrist_antagonist]
    data.kd_wrist_antagonist = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [wrist_deadband]
    data.wrist_deadband = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [wrist_pretension]
    data.wrist_pretension = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 66;
  }

  static datatype() {
    // Returns string type for a service object
    return 'roboy_middleware_msgs/JointControllerRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '8c2b216fbd74af3eb65d1ca2f72cefc3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool elbow_control_enable
    int32 kp_elbow_joint
    int32 kd_elbow_joint
    int32 kp_elbow_agonist
    int32 kd_elbow_agonist
    int32 kp_elbow_antagonist
    int32 kd_elbow_antagonist
    int32 elbow_deadband
    int32 elbow_pretension
    bool wrist_control_enable
    int32 kp_wrist_joint
    int32 kd_wrist_joint
    int32 kp_wrist_agonist
    int32 kd_wrist_agonist
    int32 kp_wrist_antagonist
    int32 kd_wrist_antagonist
    int32 wrist_deadband
    int32 wrist_pretension
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new JointControllerRequest(null);
    if (msg.elbow_control_enable !== undefined) {
      resolved.elbow_control_enable = msg.elbow_control_enable;
    }
    else {
      resolved.elbow_control_enable = false
    }

    if (msg.kp_elbow_joint !== undefined) {
      resolved.kp_elbow_joint = msg.kp_elbow_joint;
    }
    else {
      resolved.kp_elbow_joint = 0
    }

    if (msg.kd_elbow_joint !== undefined) {
      resolved.kd_elbow_joint = msg.kd_elbow_joint;
    }
    else {
      resolved.kd_elbow_joint = 0
    }

    if (msg.kp_elbow_agonist !== undefined) {
      resolved.kp_elbow_agonist = msg.kp_elbow_agonist;
    }
    else {
      resolved.kp_elbow_agonist = 0
    }

    if (msg.kd_elbow_agonist !== undefined) {
      resolved.kd_elbow_agonist = msg.kd_elbow_agonist;
    }
    else {
      resolved.kd_elbow_agonist = 0
    }

    if (msg.kp_elbow_antagonist !== undefined) {
      resolved.kp_elbow_antagonist = msg.kp_elbow_antagonist;
    }
    else {
      resolved.kp_elbow_antagonist = 0
    }

    if (msg.kd_elbow_antagonist !== undefined) {
      resolved.kd_elbow_antagonist = msg.kd_elbow_antagonist;
    }
    else {
      resolved.kd_elbow_antagonist = 0
    }

    if (msg.elbow_deadband !== undefined) {
      resolved.elbow_deadband = msg.elbow_deadband;
    }
    else {
      resolved.elbow_deadband = 0
    }

    if (msg.elbow_pretension !== undefined) {
      resolved.elbow_pretension = msg.elbow_pretension;
    }
    else {
      resolved.elbow_pretension = 0
    }

    if (msg.wrist_control_enable !== undefined) {
      resolved.wrist_control_enable = msg.wrist_control_enable;
    }
    else {
      resolved.wrist_control_enable = false
    }

    if (msg.kp_wrist_joint !== undefined) {
      resolved.kp_wrist_joint = msg.kp_wrist_joint;
    }
    else {
      resolved.kp_wrist_joint = 0
    }

    if (msg.kd_wrist_joint !== undefined) {
      resolved.kd_wrist_joint = msg.kd_wrist_joint;
    }
    else {
      resolved.kd_wrist_joint = 0
    }

    if (msg.kp_wrist_agonist !== undefined) {
      resolved.kp_wrist_agonist = msg.kp_wrist_agonist;
    }
    else {
      resolved.kp_wrist_agonist = 0
    }

    if (msg.kd_wrist_agonist !== undefined) {
      resolved.kd_wrist_agonist = msg.kd_wrist_agonist;
    }
    else {
      resolved.kd_wrist_agonist = 0
    }

    if (msg.kp_wrist_antagonist !== undefined) {
      resolved.kp_wrist_antagonist = msg.kp_wrist_antagonist;
    }
    else {
      resolved.kp_wrist_antagonist = 0
    }

    if (msg.kd_wrist_antagonist !== undefined) {
      resolved.kd_wrist_antagonist = msg.kd_wrist_antagonist;
    }
    else {
      resolved.kd_wrist_antagonist = 0
    }

    if (msg.wrist_deadband !== undefined) {
      resolved.wrist_deadband = msg.wrist_deadband;
    }
    else {
      resolved.wrist_deadband = 0
    }

    if (msg.wrist_pretension !== undefined) {
      resolved.wrist_pretension = msg.wrist_pretension;
    }
    else {
      resolved.wrist_pretension = 0
    }

    return resolved;
    }
};

class JointControllerResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type JointControllerResponse
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type JointControllerResponse
    let len;
    let data = new JointControllerResponse(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'roboy_middleware_msgs/JointControllerResponse';
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
    const resolved = new JointControllerResponse(null);
    return resolved;
    }
};

module.exports = {
  Request: JointControllerRequest,
  Response: JointControllerResponse,
  md5sum() { return '8c2b216fbd74af3eb65d1ca2f72cefc3'; },
  datatype() { return 'roboy_middleware_msgs/JointController'; }
};
