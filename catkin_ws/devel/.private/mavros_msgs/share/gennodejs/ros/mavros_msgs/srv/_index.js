
"use strict";

let FileTruncate = require('./FileTruncate.js')
let CommandBool = require('./CommandBool.js')
let FileOpen = require('./FileOpen.js')
let LogRequestEnd = require('./LogRequestEnd.js')
let CommandInt = require('./CommandInt.js')
let CommandHome = require('./CommandHome.js')
let CommandTOL = require('./CommandTOL.js')
let CommandTriggerInterval = require('./CommandTriggerInterval.js')
let WaypointClear = require('./WaypointClear.js')
let FileChecksum = require('./FileChecksum.js')
let ParamGet = require('./ParamGet.js')
let FileList = require('./FileList.js')
let FileMakeDir = require('./FileMakeDir.js')
let WaypointPull = require('./WaypointPull.js')
let WaypointPush = require('./WaypointPush.js')
let FileClose = require('./FileClose.js')
let ParamPull = require('./ParamPull.js')
let SetMode = require('./SetMode.js')
let FileRename = require('./FileRename.js')
let ParamSet = require('./ParamSet.js')
let LogRequestData = require('./LogRequestData.js')
let ParamPush = require('./ParamPush.js')
let VehicleInfoGet = require('./VehicleInfoGet.js')
let SetMavFrame = require('./SetMavFrame.js')
let FileWrite = require('./FileWrite.js')
let CommandVtolTransition = require('./CommandVtolTransition.js')
let LogRequestList = require('./LogRequestList.js')
let FileRemoveDir = require('./FileRemoveDir.js')
let CommandTriggerControl = require('./CommandTriggerControl.js')
let FileRead = require('./FileRead.js')
let MountConfigure = require('./MountConfigure.js')
let WaypointSetCurrent = require('./WaypointSetCurrent.js')
let MessageInterval = require('./MessageInterval.js')
let FileRemove = require('./FileRemove.js')
let StreamRate = require('./StreamRate.js')
let CommandLong = require('./CommandLong.js')

module.exports = {
  FileTruncate: FileTruncate,
  CommandBool: CommandBool,
  FileOpen: FileOpen,
  LogRequestEnd: LogRequestEnd,
  CommandInt: CommandInt,
  CommandHome: CommandHome,
  CommandTOL: CommandTOL,
  CommandTriggerInterval: CommandTriggerInterval,
  WaypointClear: WaypointClear,
  FileChecksum: FileChecksum,
  ParamGet: ParamGet,
  FileList: FileList,
  FileMakeDir: FileMakeDir,
  WaypointPull: WaypointPull,
  WaypointPush: WaypointPush,
  FileClose: FileClose,
  ParamPull: ParamPull,
  SetMode: SetMode,
  FileRename: FileRename,
  ParamSet: ParamSet,
  LogRequestData: LogRequestData,
  ParamPush: ParamPush,
  VehicleInfoGet: VehicleInfoGet,
  SetMavFrame: SetMavFrame,
  FileWrite: FileWrite,
  CommandVtolTransition: CommandVtolTransition,
  LogRequestList: LogRequestList,
  FileRemoveDir: FileRemoveDir,
  CommandTriggerControl: CommandTriggerControl,
  FileRead: FileRead,
  MountConfigure: MountConfigure,
  WaypointSetCurrent: WaypointSetCurrent,
  MessageInterval: MessageInterval,
  FileRemove: FileRemove,
  StreamRate: StreamRate,
  CommandLong: CommandLong,
};
