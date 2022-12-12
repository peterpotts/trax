package com.peterpotts.trax.mavlink.model

import Autopilot._
import com.peterpotts.trax.mavlink.model.MAVEnums._
import me.drton.jmavlib.mavlink.MAVLinkMessage

/**
  * @author Peter Potts
  */
object MAVExamples {
  val heartbeat: Heartbeat =
    Heartbeat(
      `type` = MAVType.Quadrotor,
      autopilot = MAVAutopilot.Generic,
      baseMode = Set(
        MAVModeFlag.ManualInputEnabled,
        MAVModeFlag.StabilizeEnabled
      ),
      customMode = 0,
      systemStatus = MAVState.Standby
    )

  val attitudeTarget: AttitudeTarget =
    AttitudeTarget(
      timeBootMS = 1852,
      typeMask = 0,
      q = IndexedSeq(0.9999997f, 4.8139373e-5f, 1.5386473e-4f, -7.058486e-4f),
      bodyRollRate = -5.947007e-7f,
      bodyPitchRate = -0.0023190193f,
      bodyYawRate = 0f,
      thrust = 0f
    )

  val altitude: Altitude =
    Altitude(
      timeUSec = 1136000L,
      altitudeMonotonic = 487.97604f,
      altitudeAMSL = Float.NaN,
      altitudeLocal = Float.NaN,
      altitudeRelative = Float.NaN,
      altitudeTerrain = Float.NaN,
      bottomClearance = Float.NaN
    )

  val attitude: Attitude =
    Attitude(
      timeBootMS = 1908,
      roll = 8.293464e-5f,
      pitch = 1.4160939e-4f,
      yaw = -0.001663092f,
      rollSpeed = 0.0023668504f,
      pitchSpeed = -0.0037892442f,
      yawSpeed = -0.0032421406f
    )

  val attitudeQuaternion: AttitudeQuaternion =
    AttitudeQuaternion(
      timeBootMS = 1824,
      q1 = 0.9999997f,
      q2 = 4.8139373e-5f,
      q3 = 1.5386473e-4f,
      q4 = -7.058486E-4f,
      rollSpeed = -0.0020403233f,
      pitchSpeed = -0.0028311175f,
      yawSpeed = 6.3513225e-4f
    )

  val extendedSysState: ExtendedSysState =
    ExtendedSysState(
      vTOLState = MAVVTOLState.Undefined,
      landedState = MAVLandedState.OnGround
    )

  val gPSRawInt: GPSRawInt =
    GPSRawInt(
      timeUSec = 1216000L,
      fixType = GPSFixType.NoGPS,
      lat = 473977419,
      lon = 85455939,
      alt = 487997,
      epH = 0,
      epV = 0,
      vel = 0,
      cOG = 0,
      satellitesVisible = 10,
      altEllipsoid = 0,
      hAcc = 0,
      vAcc = 0,
      velAcc = 0,
      hdgAcc = 0
    )

  val localPositionNED: LocalPositionNED =
    LocalPositionNED(
      timeBootMS = 1824,
      x = -4.025555e-5f,
      y = 2.468301e-5f,
      z = -0.0049281064f,
      vX = -4.6635623e-4f,
      vY = 7.222806e-5f,
      vZ = 2.3506744e-5f
    )

  val servoOutputRaw: ServoOutputRaw =
    ServoOutputRaw(
      timeUSec = 1832000,
      port = 0,
      servo1Raw = 900,
      servo2Raw = 900,
      servo3Raw = 900,
      servo4Raw = 900,
      servo5Raw = 900,
      servo6Raw = 900,
      servo7Raw = 0,
      servo8Raw = 0,
      servo9Raw = 0,
      servo10Raw = 0,
      servo11Raw = 0,
      servo12Raw = 0,
      servo13Raw = 0,
      servo14Raw = 0,
      servo15Raw = 0,
      servo16Raw = 0
    )

  val sysStatus: SysStatus =
    SysStatus(
      onboardControlSensorsPresent = Set(
        MAVSysStatusSensor.Sensor3dGyro,
        MAVSysStatusSensor.Sensor3dAccel,
        MAVSysStatusSensor.Sensor3dMag,
        MAVSysStatusSensor.SensorAbsolutePressure,
        MAVSysStatusSensor.SensorGPS,
        MAVSysStatusSensor.SensorBattery,
      ),
      onboardControlSensorsEnabled = Set(
        MAVSysStatusSensor.Sensor3dGyro,
        MAVSysStatusSensor.Sensor3dAccel,
        MAVSysStatusSensor.Sensor3dMag,
        MAVSysStatusSensor.SensorAbsolutePressure,
        MAVSysStatusSensor.SensorGPS,
        MAVSysStatusSensor.SensorRCReceiver,
        MAVSysStatusSensor.AHRS,
        MAVSysStatusSensor.SensorBattery
      ),
      onboardControlSensorsHealth = Set(
        MAVSysStatusSensor.Sensor3dGyro,
        MAVSysStatusSensor.Sensor3dAccel,
        MAVSysStatusSensor.Sensor3dMag,
        MAVSysStatusSensor.SensorAbsolutePressure,
        MAVSysStatusSensor.SensorGPS
      ),
      load = 0,
      voltageBattery = 12150,
      currentBattery = -100,
      batteryRemaining = 100,
      dropRateComm = 0,
      errorsComm = 0,
      errorsCount1 = 0,
      errorsCount2 = 0,
      errorsCount3 = 0,
      errorsCount4 = 0
    )

  val systemTime: SystemTime =
    SystemTime(
      timeUnixUSec = 23432L,
      timeBootMS = 43234
    )

  val vFRHUD: VFRHUD =
    VFRHUD(
      airspeed = 0f,
      groundSpeed = 0.0029535259f,
      heading = 359,
      throttle = 0,
      alt = 488.05118f,
      climb = 0.0016329772f
    )

  val messages: IndexedSeq[MAVLinkMessage] =
    IndexedSeq(
      attitudeTarget,
      altitude,
      attitude,
      attitudeQuaternion,
      extendedSysState,
      gPSRawInt,
      localPositionNED,
      servoOutputRaw,
      sysStatus,
      systemTime,
      vFRHUD
    )
}
