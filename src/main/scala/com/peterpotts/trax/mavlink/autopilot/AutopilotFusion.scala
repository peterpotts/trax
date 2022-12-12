package com.peterpotts.trax.mavlink.autopilot

import com.peterpotts.trax.common.{Logging, TimeUSec}
import com.peterpotts.trax.mavlink.autopilot.HILStateQuaternionDecorator._
import com.peterpotts.trax.mavlink.autopilot.ManualControlDecorator._
import com.peterpotts.trax.mavlink.model.Autopilot
import com.peterpotts.trax.mavlink.model.MAVEnums._

import scala.math._

/**
  * @author Peter Potts
  */
object AutopilotFusion extends Logging {
  def fuse(): Unit =
    for (systemTime <- AutopilotInputState.VarSystemTime.get(-1)) {
      logger.info("Fusing...")
      val timeUSec = TimeUSec.now()

      val timeBootMS = {
        val deltaUSec = timeUSec - systemTime.timeUnixUSec
        val deltaMS = (deltaUSec / 1000L).toInt
        systemTime.timeBootMS + deltaMS
      }

      /**
        * For fusion use:
        * AutopilotMeasurementState.VarHILGPS
        * AutopilotMeasurementState.VarHILStateQuaternion
        */
      val fusedLat: Double = 0 // degrees
      val fusedLon: Double = 0 // degrees

      val originDisN: Double = 0 // meters
      val originDisE: Double = 0 // meters
      val originDisD: Double = 0 // meters

      val homeDisN: Double = 0 // meters
      val homeDisE: Double = 0 // meters
      val homeDisD: Double = 0 // meters

      val fusedDisN: Double = 0 // meters
      val fusedDisE: Double = 0 // meters
      val fusedDisD: Double = 0 // meters

      // Uncorrected barometric altitude at boot time
      val altitudeMonotonic: Double = 0 // meters

      //val fusedAltWGS84: Double = 0 // meters

      val fusedAltAMSL: Double = 0 // meters above mean sea level
      val altitudeLocal: Double = fusedAltAMSL + originDisD // meters
      val fusedRelativeAlt: Double = fusedAltAMSL + homeDisD // meters
      val altitudeTerrain: Double = -1000 // meters

      // Laser altimeter
      val fusedBottomClearance: Double = -1.0 // meters

      val fusedVelN: Double = 0 // meters per second
      val fusedVelE: Double = 0 // meters per second
      val fusedVelD: Double = 0 // meters per second

      val fusedRoll: Double = 0 // radians [-Pi to Pi]
      val fusedPitch: Double = 0 // radians [-Pi to Pi]
      val fusedYaw: Double = 0 // radians [-Pi to Pi]
      val fusedRollSpeed: Double = 0 // radians per second
      val fusedPitchSpeed: Double = 0 // radians per second
      val fusedYawSpeed: Double = 0 // radians per second

      val fusedHeading: Double =
        if (fusedYaw < 0) fusedYaw.toDegrees + 360 else fusedYaw.toDegrees

      val fusedDisX: Double = fusedDisN
      val fusedDisY: Double = fusedDisE
      val fusedDisZ: Double = fusedDisD

      val fusedVelX: Double = fusedVelN
      val fusedVelY: Double = fusedVelE
      val fusedVelZ: Double = fusedVelD

      val fusedAirspeed: Double = 0 // meters per second
      val fusedGroundSpeed: Double = sqrt(fusedDisN * fusedDisN + fusedDisE * fusedDisE)
      val throttle: Int = 0 // 0 to 100
      val fusedClimbRate: Double = -fusedVelD

      val gPSPositionUncertainty: Double = 0.3 // meters
      val gPSAltitudeUncertainty: Double = 0.4 // meters
      val gPSSpeedUncertainty: Double = 0.25 // meters per second
      val gPSHeadingUncertainty: Double = 0.1 // degrees

      AutopilotOutputState.VarSysStatus.set(
        Autopilot.SysStatus(
          onboardControlSensorsPresent = Set(MAVSysStatusSensor.SensorGPS, MAVSysStatusSensor.SensorBattery),
          onboardControlSensorsEnabled = Set(MAVSysStatusSensor.SensorGPS, MAVSysStatusSensor.SensorBattery),
          onboardControlSensorsHealth = Set(MAVSysStatusSensor.SensorGPS, MAVSysStatusSensor.SensorBattery),
          load = 10, // 1 = 0.1%
          voltageBattery = 11100, // 1 = 1 millivolt
          currentBattery = -1, // 1 = 10 milliampere
          batteryRemaining = 64, // 1 = 1%
          dropRateComm = 3, // 1 = 0.01%
          errorsComm = 1,
          errorsCount1 = 0,
          errorsCount2 = 0,
          errorsCount3 = 0,
          errorsCount4 = 0
        )
      )

      AutopilotOutputState.VarAltitude.set(
        Autopilot.Altitude(
          timeUSec = timeUSec,
          altitudeMonotonic = altitudeMonotonic.toFloat,
          altitudeAMSL = fusedAltAMSL.toFloat,
          altitudeLocal = altitudeLocal.toFloat,
          altitudeRelative = fusedRelativeAlt.toFloat,
          altitudeTerrain = altitudeTerrain.toFloat,
          bottomClearance = fusedBottomClearance.toFloat
        )
      )

      for (hILStateQuaternion <- AutopilotInputState.VarHILStateQuaternion.get(0))
        AutopilotOutputState.VarAttitudeQuaternion.set(
          Autopilot.AttitudeQuaternion(
            timeBootMS = timeBootMS,
            q1 = hILStateQuaternion.qW,
            q2 = hILStateQuaternion.qX,
            q3 = hILStateQuaternion.qY,
            q4 = hILStateQuaternion.qZ,
            rollSpeed = hILStateQuaternion.rollSpeed,
            pitchSpeed = hILStateQuaternion.pitchSpeed,
            yawSpeed = hILStateQuaternion.yawSpeed
          )
        )

      AutopilotOutputState.VarGlobalPositionInt.set(
        Autopilot.GlobalPositionInt(
          timeBootMS = timeBootMS,
          lat = (fusedLat * 1e7).toInt,
          lon = (fusedLon * 1e7).toInt,
          alt = (fusedAltAMSL * 1000).toInt,
          relativeAlt = (fusedRelativeAlt * 1000).toInt,
          vX = (fusedVelX * 100).toInt,
          vY = (fusedVelY * 100).toInt,
          vZ = (fusedVelZ * 100).toInt,
          hdg = (fusedHeading * 100).toInt
        )
      )

      for (hILGPS <- AutopilotInputState.VarHILGPS.get(0))
        AutopilotOutputState.VarGPSRawInt.set(
          Autopilot.GPSRawInt(
            timeUSec = hILGPS.timeUSec,
            fixType = GPSFixType(hILGPS.fixType),
            lat = hILGPS.lat,
            lon = hILGPS.lon,
            alt = hILGPS.alt,
            epH = hILGPS.epH,
            epV = hILGPS.epV,
            vel = hILGPS.vel,
            cOG = hILGPS.cOG,
            satellitesVisible = hILGPS.satellitesVisible,
            altEllipsoid = hILGPS.alt,
            hAcc = (gPSPositionUncertainty * 1000).toInt,
            vAcc = (gPSAltitudeUncertainty * 1000).toInt,
            velAcc = (gPSSpeedUncertainty * 1000).toInt,
            hdgAcc = (gPSHeadingUncertainty * 1e5).toInt
          )
        )

      AutopilotOutputState.VarLocalPositionNED.set(
        Autopilot.LocalPositionNED(
          timeBootMS = timeBootMS,
          x = fusedDisX.toFloat,
          y = fusedDisY.toFloat,
          z = fusedDisZ.toFloat,
          vX = fusedVelX.toFloat,
          vY = fusedVelY.toFloat,
          vZ = fusedVelZ.toFloat
        )
      )

      AutopilotOutputState.VarVFRHUD.set(
        Autopilot.VFRHUD(
          airspeed = fusedAirspeed.toFloat,
          groundSpeed = fusedGroundSpeed.toFloat,
          heading = fusedHeading.toInt,
          throttle = throttle,
          alt = fusedAltAMSL.toFloat,
          climb = fusedClimbRate.toFloat
        )
      )

      AutopilotOutputState.VarAttitude.set(
        Autopilot.Attitude(
          timeBootMS = timeBootMS,
          roll = fusedRoll.toFloat,
          pitch = fusedPitch.toFloat,
          yaw = fusedYaw.toFloat,
          rollSpeed = fusedRollSpeed.toFloat,
          pitchSpeed = fusedPitchSpeed.toFloat,
          yawSpeed = fusedYawSpeed.toFloat
        )
      )

      for (landedState <- AutopilotControlState.VarLandedState.get(0))
        AutopilotOutputState.VarExtendedSysState.set(
          Autopilot.ExtendedSysState(
            vTOLState = MAVVTOLState.Undefined,
            landedState = landedState
          )
        )

      /////////////////////////////////
      /////////////////////////////////
      // TODO WORK IN PROGRESS BELOW //
      /////////////////////////////////
      /////////////////////////////////

      for (hILRCInputsRaw <- AutopilotControlState.VarHILRCInputsRaw.get(1))
        AutopilotOutputState.VarRCChannels.set(
          Autopilot.RCChannels(
            timeBootMS = hILRCInputsRaw.timeUSec.toInt,
            chanCount = 12,
            chan1Raw = hILRCInputsRaw.chan1Raw,
            chan2Raw = hILRCInputsRaw.chan2Raw,
            chan3Raw = hILRCInputsRaw.chan3Raw,
            chan4Raw = hILRCInputsRaw.chan4Raw,
            chan5Raw = hILRCInputsRaw.chan5Raw,
            chan6Raw = hILRCInputsRaw.chan6Raw,
            chan7Raw = hILRCInputsRaw.chan7Raw,
            chan8Raw = hILRCInputsRaw.chan8Raw,
            chan9Raw = hILRCInputsRaw.chan9Raw,
            chan10Raw = hILRCInputsRaw.chan10Raw,
            chan11Raw = hILRCInputsRaw.chan11Raw,
            chan12Raw = hILRCInputsRaw.chan12Raw,
            chan13Raw = 0,
            chan14Raw = 0,
            chan15Raw = 0,
            chan16Raw = 0,
            chan17Raw = 0,
            chan18Raw = 0,
            rSSI = hILRCInputsRaw.rSSI
          )
        )

      for (manualControl <- AutopilotControlState.VarManualControl.get(0))
        AutopilotOutputState.VarServoOutputRaw.set(
          Autopilot.ServoOutputRaw(
            timeUSec = timeBootMS * 1000,
            port = 1,
            servo1Raw = 1000 + manualControl.throttle,
            servo2Raw = 1000 + manualControl.throttle,
            servo3Raw = 1000 + manualControl.throttle,
            servo4Raw = 1000 + manualControl.throttle,
            servo5Raw = 0,
            servo6Raw = 0,
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
        )

      AutopilotOutputState.VarHILActuatorControls.set(
        Autopilot.HILActuatorControls(
          timeUSec = timeUSec,
          controls = IndexedSeq(0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f),
          mode = MAVMode.ManualArmed,
          flags = 1L
        )
      )
    }
}
