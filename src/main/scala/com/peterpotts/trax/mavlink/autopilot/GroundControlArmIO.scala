package com.peterpotts.trax.mavlink.autopilot

import com.peterpotts.trax.mavlink.model.MAVEnums._
import com.peterpotts.trax.mavlink.model._

/**
  * Read from Vehicle.
  * Write from Autopilot.
  *
  * @author Peter Potts
  */
case class GroundControlArmIO(write: MAVMessage => Unit) extends AutopilotGroundControl {
  private val componentArm =
    GroundControl.CommandLong(
      targetSystem = systemId,
      targetComponent = localId,
      command = MAVCmd.ComponentArmDisarm,
      confirmation = 0,
      param1 = 1f,
      param2 = 0f,
      param3 = 0f,
      param4 = 0f,
      param5 = 0f,
      param6 = 0f,
      param7 = 0f
    )

  private val componentDisarm =
    GroundControl.CommandLong(
      targetSystem = systemId,
      targetComponent = localId,
      command = MAVCmd.ComponentArmDisarm,
      confirmation = 0,
      param1 = 0f,
      param2 = 0f,
      param3 = 0f,
      param4 = 0f,
      param5 = 0f,
      param6 = 0f,
      param7 = 0f
    )

  private val commandAck =
    Autopilot.CommandAck(
      command = MAVCmd.ComponentArmDisarm,
      result = MAVResult.Accepted,
      progress = 0,
      resultParam2 = 0,
      targetSystem = systemId,
      targetComponent = remoteId
    )

  val read: PartialFunction[MAVMessage, Unit] = {
    case `componentArm` =>
      logger.info("GroundControl> READ ComponentArm")
      AutopilotAutomatonState.armed = true
      logger.info("GroundControl> WRITE CommandAck")
      write(commandAck)
    case `componentDisarm` =>
      logger.info("GroundControl> READ ComponentDisarm")
      AutopilotAutomatonState.armed = false
      logger.info("GroundControl> WRITE CommandAck")
      write(commandAck)
  }
}
