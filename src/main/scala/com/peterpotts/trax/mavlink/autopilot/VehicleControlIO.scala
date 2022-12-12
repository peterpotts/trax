package com.peterpotts.trax.mavlink.autopilot

import com.peterpotts.trax.mavlink.model._

/**
  * Read from Vehicle.
  * Write from Autopilot.
  *
  * @author Peter Potts
  */
case class VehicleControlIO(write: MAVMessage => Unit) extends AutopilotVehicleControl {
  def tick(sequenceNumber: Int): Unit =
    AutopilotOutputState.VarHILActuatorControls.get(0).foreach { hILActuatorControls =>
      logger.info(s"Vehicle:Tick:$sequenceNumber> WRITE HILActuatorControls")
      write(hILActuatorControls)
    }

  val read: PartialFunction[MAVMessage, Unit] = {
    case hILRCInputsRaw: Vehicle.HILRCInputsRaw =>
      logger.info(s"Vehicle> READ HILRCInputsRaw")
      AutopilotControlState.VarHILRCInputsRaw.set(hILRCInputsRaw)
  }
}
