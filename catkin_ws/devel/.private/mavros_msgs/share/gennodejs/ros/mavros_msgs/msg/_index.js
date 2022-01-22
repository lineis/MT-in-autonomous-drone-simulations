
"use strict";

let ESCInfoItem = require('./ESCInfoItem.js');
let OnboardComputerStatus = require('./OnboardComputerStatus.js');
let DebugValue = require('./DebugValue.js');
let GPSRTK = require('./GPSRTK.js');
let Waypoint = require('./Waypoint.js');
let Trajectory = require('./Trajectory.js');
let GlobalPositionTarget = require('./GlobalPositionTarget.js');
let ESCInfo = require('./ESCInfo.js');
let HilStateQuaternion = require('./HilStateQuaternion.js');
let CommandCode = require('./CommandCode.js');
let ExtendedState = require('./ExtendedState.js');
let VFR_HUD = require('./VFR_HUD.js');
let HilGPS = require('./HilGPS.js');
let GPSRAW = require('./GPSRAW.js');
let LogData = require('./LogData.js');
let StatusText = require('./StatusText.js');
let Thrust = require('./Thrust.js');
let OpticalFlowRad = require('./OpticalFlowRad.js');
let ESCStatus = require('./ESCStatus.js');
let PositionTarget = require('./PositionTarget.js');
let Param = require('./Param.js');
let EstimatorStatus = require('./EstimatorStatus.js');
let CamIMUStamp = require('./CamIMUStamp.js');
let HilControls = require('./HilControls.js');
let HilSensor = require('./HilSensor.js');
let Mavlink = require('./Mavlink.js');
let HilActuatorControls = require('./HilActuatorControls.js');
let RCIn = require('./RCIn.js');
let LogEntry = require('./LogEntry.js');
let ESCStatusItem = require('./ESCStatusItem.js');
let HomePosition = require('./HomePosition.js');
let Altitude = require('./Altitude.js');
let BatteryStatus = require('./BatteryStatus.js');
let ManualControl = require('./ManualControl.js');
let RCOut = require('./RCOut.js');
let CompanionProcessStatus = require('./CompanionProcessStatus.js');
let WheelOdomStamped = require('./WheelOdomStamped.js');
let PlayTuneV2 = require('./PlayTuneV2.js');
let VehicleInfo = require('./VehicleInfo.js');
let RTKBaseline = require('./RTKBaseline.js');
let LandingTarget = require('./LandingTarget.js');
let AttitudeTarget = require('./AttitudeTarget.js');
let MountControl = require('./MountControl.js');
let ActuatorControl = require('./ActuatorControl.js');
let State = require('./State.js');
let RadioStatus = require('./RadioStatus.js');
let WaypointList = require('./WaypointList.js');
let RTCM = require('./RTCM.js');
let TimesyncStatus = require('./TimesyncStatus.js');
let ADSBVehicle = require('./ADSBVehicle.js');
let OverrideRCIn = require('./OverrideRCIn.js');
let Vibration = require('./Vibration.js');
let FileEntry = require('./FileEntry.js');
let ParamValue = require('./ParamValue.js');
let WaypointReached = require('./WaypointReached.js');

module.exports = {
  ESCInfoItem: ESCInfoItem,
  OnboardComputerStatus: OnboardComputerStatus,
  DebugValue: DebugValue,
  GPSRTK: GPSRTK,
  Waypoint: Waypoint,
  Trajectory: Trajectory,
  GlobalPositionTarget: GlobalPositionTarget,
  ESCInfo: ESCInfo,
  HilStateQuaternion: HilStateQuaternion,
  CommandCode: CommandCode,
  ExtendedState: ExtendedState,
  VFR_HUD: VFR_HUD,
  HilGPS: HilGPS,
  GPSRAW: GPSRAW,
  LogData: LogData,
  StatusText: StatusText,
  Thrust: Thrust,
  OpticalFlowRad: OpticalFlowRad,
  ESCStatus: ESCStatus,
  PositionTarget: PositionTarget,
  Param: Param,
  EstimatorStatus: EstimatorStatus,
  CamIMUStamp: CamIMUStamp,
  HilControls: HilControls,
  HilSensor: HilSensor,
  Mavlink: Mavlink,
  HilActuatorControls: HilActuatorControls,
  RCIn: RCIn,
  LogEntry: LogEntry,
  ESCStatusItem: ESCStatusItem,
  HomePosition: HomePosition,
  Altitude: Altitude,
  BatteryStatus: BatteryStatus,
  ManualControl: ManualControl,
  RCOut: RCOut,
  CompanionProcessStatus: CompanionProcessStatus,
  WheelOdomStamped: WheelOdomStamped,
  PlayTuneV2: PlayTuneV2,
  VehicleInfo: VehicleInfo,
  RTKBaseline: RTKBaseline,
  LandingTarget: LandingTarget,
  AttitudeTarget: AttitudeTarget,
  MountControl: MountControl,
  ActuatorControl: ActuatorControl,
  State: State,
  RadioStatus: RadioStatus,
  WaypointList: WaypointList,
  RTCM: RTCM,
  TimesyncStatus: TimesyncStatus,
  ADSBVehicle: ADSBVehicle,
  OverrideRCIn: OverrideRCIn,
  Vibration: Vibration,
  FileEntry: FileEntry,
  ParamValue: ParamValue,
  WaypointReached: WaypointReached,
};
