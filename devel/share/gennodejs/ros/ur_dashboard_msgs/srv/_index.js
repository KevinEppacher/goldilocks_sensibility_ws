
"use strict";

let GetLoadedProgram = require('./GetLoadedProgram.js')
let AddToLog = require('./AddToLog.js')
let Popup = require('./Popup.js')
let GetSafetyMode = require('./GetSafetyMode.js')
let RawRequest = require('./RawRequest.js')
let IsProgramSaved = require('./IsProgramSaved.js')
let IsInRemoteControl = require('./IsInRemoteControl.js')
let GetProgramState = require('./GetProgramState.js')
let Load = require('./Load.js')
let IsProgramRunning = require('./IsProgramRunning.js')
let GetRobotMode = require('./GetRobotMode.js')

module.exports = {
  GetLoadedProgram: GetLoadedProgram,
  AddToLog: AddToLog,
  Popup: Popup,
  GetSafetyMode: GetSafetyMode,
  RawRequest: RawRequest,
  IsProgramSaved: IsProgramSaved,
  IsInRemoteControl: IsInRemoteControl,
  GetProgramState: GetProgramState,
  Load: Load,
  IsProgramRunning: IsProgramRunning,
  GetRobotMode: GetRobotMode,
};
