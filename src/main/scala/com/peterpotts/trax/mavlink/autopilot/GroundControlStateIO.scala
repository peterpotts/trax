package com.peterpotts.trax.mavlink.autopilot

import com.peterpotts.trax.mavlink.model._

/**
  * @author Peter Potts
  */
case class GroundControlStateIO(write: MAVMessage => Unit) extends AutopilotGroundControl {
  def tick(sequenceNumber: Int): Unit =
    if (AutopilotAutomatonState.initialized) {
      AutopilotFusion.fuse()

      for (sysStatus <- AutopilotOutputState.VarSysStatus.get(0)) {
        logger.trace(s"GroundControl:Tick:$sequenceNumber> WRITE SysStatus")
        write(sysStatus)
      }

      for (altitude <- AutopilotOutputState.VarAltitude.get(0)) {
        logger.trace(s"GroundControl:Tick:$sequenceNumber> WRITE Altitude")
        write(altitude)
      }

      for (attitudeQuaternion <- AutopilotOutputState.VarAttitudeQuaternion.get(0)) {
        logger.trace(s"GroundControl:Tick:$sequenceNumber> WRITE AttitudeQuaternion")
        write(attitudeQuaternion)
      }

      for (globalPositionInt <- AutopilotOutputState.VarGlobalPositionInt.get(0)) {
        logger.trace(s"GroundControl:Tick:$sequenceNumber> WRITE GlobalPositionInt")
        write(globalPositionInt)
      }

      for (localPositionNED <- AutopilotOutputState.VarLocalPositionNED.get(0)) {
        logger.trace(s"GroundControl:Tick:$sequenceNumber> WRITE LocalPositionNED")
        write(localPositionNED)
      }

      for (vFRHUD <- AutopilotOutputState.VarVFRHUD.get(0)) {
        logger.trace(s"GroundControl:Tick:$sequenceNumber> WRITE VFRHUD")
        write(vFRHUD)
      }

      for (gPSRawInt <- AutopilotOutputState.VarGPSRawInt.get(0)) {
        logger.trace(s"GroundControl:Tick:$sequenceNumber> WRITE GPSRawInt")
        write(gPSRawInt)
      }

      for (servoOutputRaw <- AutopilotOutputState.VarServoOutputRaw.get(0)) {
        logger.info(s"GroundControl:Tick:$sequenceNumber> WRITE ServoOutputRaw")
        write(servoOutputRaw)
      }

      for (rCChannels <- AutopilotOutputState.VarRCChannels.get(0)) {
        logger.info(s"GroundControl:Tick:$sequenceNumber> WRITE RCChannels")
        write(rCChannels)
      }

      for (attitude <- AutopilotOutputState.VarAttitude.get(0)) {
        logger.info(s"GroundControl:Tick:$sequenceNumber> WRITE Attitude")
        write(attitude)
      }

      for (extendedSysState <- AutopilotOutputState.VarExtendedSysState.get(0)) {
        logger.trace(s"GroundControl:Tick:$sequenceNumber> WRITE ExtendedSysState")
        write(extendedSysState)
      }
    }

  val read: PartialFunction[MAVMessage, Unit] = PartialFunction.empty
}
