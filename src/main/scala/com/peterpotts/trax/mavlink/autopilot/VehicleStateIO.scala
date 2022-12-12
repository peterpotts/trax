package com.peterpotts.trax.mavlink.autopilot

import com.peterpotts.trax.mavlink.model._

/**
  * Read from Vehicle.
  * Write from Autopilot.
  *
  * @author Peter Potts
  */
case class VehicleStateIO(write: MAVMessage => Unit) extends AutopilotVehicleControl {
  val read: PartialFunction[MAVMessage, Unit] = {
    case systemTime: Vehicle.SystemTime =>
      logger.info("Vehicle> READ SystemTime")
      AutopilotInputState.VarSystemTime.set(systemTime)
    case hILSensor: Vehicle.HILSensor =>
      logger.trace(s"Vehicle> READ HILSensor")
      AutopilotInputState.VarHILSensor.set(hILSensor)
    case hILStateQuaternion: Vehicle.HILStateQuaternion =>
      logger.trace(s"Vehicle> READ HILStateQuaternion")
      AutopilotInputState.VarHILStateQuaternion.set(hILStateQuaternion)
    case hILGPS: Vehicle.HILGPS =>
      logger.trace(s"Vehicle> READ HILGPS")
      AutopilotInputState.VarHILGPS.set(hILGPS)
  }
}
