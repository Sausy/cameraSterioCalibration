
"use strict";

let ADCvalue = require('./ADCvalue.js');
let HandCommand = require('./HandCommand.js');
let DarkRoomStatus = require('./DarkRoomStatus.js');
let MotorCommand = require('./MotorCommand.js');
let ArucoPose = require('./ArucoPose.js');
let ControllerState = require('./ControllerState.js');
let LighthousePoseCorrection = require('./LighthousePoseCorrection.js');
let JointStatus = require('./JointStatus.js');
let MotorAngle = require('./MotorAngle.js');
let DanceCommand = require('./DanceCommand.js');
let MotorStatus = require('./MotorStatus.js');
let ArmStatus = require('./ArmStatus.js');
let DarkRoomOOTX = require('./DarkRoomOOTX.js');
let MagneticSensor = require('./MagneticSensor.js');
let DarkRoomSensor = require('./DarkRoomSensor.js');
let DarkRoomStatistics = require('./DarkRoomStatistics.js');
let MotorConfig = require('./MotorConfig.js');
let DarkRoomSensorV2 = require('./DarkRoomSensorV2.js');
let DarkRoom = require('./DarkRoom.js');

module.exports = {
  ADCvalue: ADCvalue,
  HandCommand: HandCommand,
  DarkRoomStatus: DarkRoomStatus,
  MotorCommand: MotorCommand,
  ArucoPose: ArucoPose,
  ControllerState: ControllerState,
  LighthousePoseCorrection: LighthousePoseCorrection,
  JointStatus: JointStatus,
  MotorAngle: MotorAngle,
  DanceCommand: DanceCommand,
  MotorStatus: MotorStatus,
  ArmStatus: ArmStatus,
  DarkRoomOOTX: DarkRoomOOTX,
  MagneticSensor: MagneticSensor,
  DarkRoomSensor: DarkRoomSensor,
  DarkRoomStatistics: DarkRoomStatistics,
  MotorConfig: MotorConfig,
  DarkRoomSensorV2: DarkRoomSensorV2,
  DarkRoom: DarkRoom,
};
