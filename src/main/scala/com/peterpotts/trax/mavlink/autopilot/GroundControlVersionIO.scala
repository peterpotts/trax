package com.peterpotts.trax.mavlink.autopilot

import com.peterpotts.trax.mavlink.model.MAVEnums._
import com.peterpotts.trax.mavlink.model._

/**
  * @author Peter Potts
  */
case class GroundControlVersionIO(write: MAVMessage => Unit) extends AutopilotGroundControl {
  private val requestProtocolVersion =
    GroundControl.CommandLong(
      targetSystem = systemId,
      targetComponent = broadcastId,
      command = MAVCmd.RequestProtocolVersion,
      confirmation = 0,
      param1 = 1f,
      param2 = 0f,
      param3 = 0f,
      param4 = 0f,
      param5 = 0f,
      param6 = 0f,
      param7 = 0f
    )

  private val commandAck =
    Autopilot.CommandAck(
      command = MAVCmd.RequestProtocolVersion,
      result = MAVResult.Accepted,
      progress = 0,
      resultParam2 = 0,
      targetSystem = systemId,
      targetComponent = remoteId
    )

  private val protocolVersion =
    Autopilot.ProtocolVersion(
      version = 200,
      minVersion = 100,
      maxVersion = 200,
      specVersionHash = IndexedSeq(0, 0, 0, 0, 0, 0, 0, 0),
      libraryVersionHash = IndexedSeq(250, 185, 36, 223, 24, 66, 158, 15)
    )

  val read: PartialFunction[MAVMessage, Unit] = {
    case `requestProtocolVersion` =>
      logger.info("GroundControl> READ RequestProtocolVersion")
      logger.info("GroundControl> WRITE CommandAck")
      write(commandAck)
      logger.info("GroundControl> WRITE ProtocolVersion")
      write(protocolVersion)
  }
}
