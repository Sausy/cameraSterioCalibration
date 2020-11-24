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

class MotorConfig {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.id = null;
      this.motors = null;
      this.control_mode = null;
      this.output_pos_max = null;
      this.output_neg_max = null;
      this.sp_pos_max = null;
      this.sp_neg_max = null;
      this.kp = null;
      this.ki = null;
      this.kd = null;
      this.forward_gain = null;
      this.dead_band = null;
      this.integral_pos_max = null;
      this.integral_neg_max = null;
      this.output_divider = null;
      this.setpoint = null;
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
      if (initObj.hasOwnProperty('control_mode')) {
        this.control_mode = initObj.control_mode
      }
      else {
        this.control_mode = [];
      }
      if (initObj.hasOwnProperty('output_pos_max')) {
        this.output_pos_max = initObj.output_pos_max
      }
      else {
        this.output_pos_max = [];
      }
      if (initObj.hasOwnProperty('output_neg_max')) {
        this.output_neg_max = initObj.output_neg_max
      }
      else {
        this.output_neg_max = [];
      }
      if (initObj.hasOwnProperty('sp_pos_max')) {
        this.sp_pos_max = initObj.sp_pos_max
      }
      else {
        this.sp_pos_max = [];
      }
      if (initObj.hasOwnProperty('sp_neg_max')) {
        this.sp_neg_max = initObj.sp_neg_max
      }
      else {
        this.sp_neg_max = [];
      }
      if (initObj.hasOwnProperty('kp')) {
        this.kp = initObj.kp
      }
      else {
        this.kp = [];
      }
      if (initObj.hasOwnProperty('ki')) {
        this.ki = initObj.ki
      }
      else {
        this.ki = [];
      }
      if (initObj.hasOwnProperty('kd')) {
        this.kd = initObj.kd
      }
      else {
        this.kd = [];
      }
      if (initObj.hasOwnProperty('forward_gain')) {
        this.forward_gain = initObj.forward_gain
      }
      else {
        this.forward_gain = [];
      }
      if (initObj.hasOwnProperty('dead_band')) {
        this.dead_band = initObj.dead_band
      }
      else {
        this.dead_band = [];
      }
      if (initObj.hasOwnProperty('integral_pos_max')) {
        this.integral_pos_max = initObj.integral_pos_max
      }
      else {
        this.integral_pos_max = [];
      }
      if (initObj.hasOwnProperty('integral_neg_max')) {
        this.integral_neg_max = initObj.integral_neg_max
      }
      else {
        this.integral_neg_max = [];
      }
      if (initObj.hasOwnProperty('output_divider')) {
        this.output_divider = initObj.output_divider
      }
      else {
        this.output_divider = [];
      }
      if (initObj.hasOwnProperty('setpoint')) {
        this.setpoint = initObj.setpoint
      }
      else {
        this.setpoint = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MotorConfig
    // Serialize message field [id]
    bufferOffset = _serializer.int32(obj.id, buffer, bufferOffset);
    // Serialize message field [motors]
    bufferOffset = _arraySerializer.uint8(obj.motors, buffer, bufferOffset, null);
    // Serialize message field [control_mode]
    bufferOffset = _arraySerializer.uint8(obj.control_mode, buffer, bufferOffset, null);
    // Serialize message field [output_pos_max]
    bufferOffset = _arraySerializer.int32(obj.output_pos_max, buffer, bufferOffset, null);
    // Serialize message field [output_neg_max]
    bufferOffset = _arraySerializer.int32(obj.output_neg_max, buffer, bufferOffset, null);
    // Serialize message field [sp_pos_max]
    bufferOffset = _arraySerializer.int32(obj.sp_pos_max, buffer, bufferOffset, null);
    // Serialize message field [sp_neg_max]
    bufferOffset = _arraySerializer.int32(obj.sp_neg_max, buffer, bufferOffset, null);
    // Serialize message field [kp]
    bufferOffset = _arraySerializer.uint16(obj.kp, buffer, bufferOffset, null);
    // Serialize message field [ki]
    bufferOffset = _arraySerializer.uint16(obj.ki, buffer, bufferOffset, null);
    // Serialize message field [kd]
    bufferOffset = _arraySerializer.uint16(obj.kd, buffer, bufferOffset, null);
    // Serialize message field [forward_gain]
    bufferOffset = _arraySerializer.uint16(obj.forward_gain, buffer, bufferOffset, null);
    // Serialize message field [dead_band]
    bufferOffset = _arraySerializer.uint16(obj.dead_band, buffer, bufferOffset, null);
    // Serialize message field [integral_pos_max]
    bufferOffset = _arraySerializer.int32(obj.integral_pos_max, buffer, bufferOffset, null);
    // Serialize message field [integral_neg_max]
    bufferOffset = _arraySerializer.int32(obj.integral_neg_max, buffer, bufferOffset, null);
    // Serialize message field [output_divider]
    bufferOffset = _arraySerializer.int32(obj.output_divider, buffer, bufferOffset, null);
    // Serialize message field [setpoint]
    bufferOffset = _arraySerializer.int32(obj.setpoint, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MotorConfig
    let len;
    let data = new MotorConfig(null);
    // Deserialize message field [id]
    data.id = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [motors]
    data.motors = _arrayDeserializer.uint8(buffer, bufferOffset, null)
    // Deserialize message field [control_mode]
    data.control_mode = _arrayDeserializer.uint8(buffer, bufferOffset, null)
    // Deserialize message field [output_pos_max]
    data.output_pos_max = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [output_neg_max]
    data.output_neg_max = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [sp_pos_max]
    data.sp_pos_max = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [sp_neg_max]
    data.sp_neg_max = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [kp]
    data.kp = _arrayDeserializer.uint16(buffer, bufferOffset, null)
    // Deserialize message field [ki]
    data.ki = _arrayDeserializer.uint16(buffer, bufferOffset, null)
    // Deserialize message field [kd]
    data.kd = _arrayDeserializer.uint16(buffer, bufferOffset, null)
    // Deserialize message field [forward_gain]
    data.forward_gain = _arrayDeserializer.uint16(buffer, bufferOffset, null)
    // Deserialize message field [dead_band]
    data.dead_band = _arrayDeserializer.uint16(buffer, bufferOffset, null)
    // Deserialize message field [integral_pos_max]
    data.integral_pos_max = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [integral_neg_max]
    data.integral_neg_max = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [output_divider]
    data.output_divider = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [setpoint]
    data.setpoint = _arrayDeserializer.int32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.motors.length;
    length += object.control_mode.length;
    length += 4 * object.output_pos_max.length;
    length += 4 * object.output_neg_max.length;
    length += 4 * object.sp_pos_max.length;
    length += 4 * object.sp_neg_max.length;
    length += 2 * object.kp.length;
    length += 2 * object.ki.length;
    length += 2 * object.kd.length;
    length += 2 * object.forward_gain.length;
    length += 2 * object.dead_band.length;
    length += 4 * object.integral_pos_max.length;
    length += 4 * object.integral_neg_max.length;
    length += 4 * object.output_divider.length;
    length += 4 * object.setpoint.length;
    return length + 64;
  }

  static datatype() {
    // Returns string type for a message object
    return 'roboy_middleware_msgs/MotorConfig';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c59b5188ed0d7529ac35ce108ce30bbc';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new MotorConfig(null);
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

    if (msg.control_mode !== undefined) {
      resolved.control_mode = msg.control_mode;
    }
    else {
      resolved.control_mode = []
    }

    if (msg.output_pos_max !== undefined) {
      resolved.output_pos_max = msg.output_pos_max;
    }
    else {
      resolved.output_pos_max = []
    }

    if (msg.output_neg_max !== undefined) {
      resolved.output_neg_max = msg.output_neg_max;
    }
    else {
      resolved.output_neg_max = []
    }

    if (msg.sp_pos_max !== undefined) {
      resolved.sp_pos_max = msg.sp_pos_max;
    }
    else {
      resolved.sp_pos_max = []
    }

    if (msg.sp_neg_max !== undefined) {
      resolved.sp_neg_max = msg.sp_neg_max;
    }
    else {
      resolved.sp_neg_max = []
    }

    if (msg.kp !== undefined) {
      resolved.kp = msg.kp;
    }
    else {
      resolved.kp = []
    }

    if (msg.ki !== undefined) {
      resolved.ki = msg.ki;
    }
    else {
      resolved.ki = []
    }

    if (msg.kd !== undefined) {
      resolved.kd = msg.kd;
    }
    else {
      resolved.kd = []
    }

    if (msg.forward_gain !== undefined) {
      resolved.forward_gain = msg.forward_gain;
    }
    else {
      resolved.forward_gain = []
    }

    if (msg.dead_band !== undefined) {
      resolved.dead_band = msg.dead_band;
    }
    else {
      resolved.dead_band = []
    }

    if (msg.integral_pos_max !== undefined) {
      resolved.integral_pos_max = msg.integral_pos_max;
    }
    else {
      resolved.integral_pos_max = []
    }

    if (msg.integral_neg_max !== undefined) {
      resolved.integral_neg_max = msg.integral_neg_max;
    }
    else {
      resolved.integral_neg_max = []
    }

    if (msg.output_divider !== undefined) {
      resolved.output_divider = msg.output_divider;
    }
    else {
      resolved.output_divider = []
    }

    if (msg.setpoint !== undefined) {
      resolved.setpoint = msg.setpoint;
    }
    else {
      resolved.setpoint = []
    }

    return resolved;
    }
};

module.exports = MotorConfig;
