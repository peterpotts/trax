package com.peterpotts.trax.mavlink.autopilot

import com.peterpotts.trax.mavlink.model.MAVEnums._
import com.peterpotts.trax.mavlink.model._

/**
  * @author Peter Potts
  */
case class GroundControlCapabilitiesIO(write: MAVMessage => Unit) extends AutopilotGroundControl {
  private val requestAutopilotCapabilities =
    GroundControl.CommandLong(
      targetSystem = systemId,
      targetComponent = broadcastId,
      command = MAVCmd.RequestAutopilotCapabilities,
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
      command = MAVCmd.RequestAutopilotCapabilities,
      result = MAVResult.Accepted,
      progress = 0,
      resultParam2 = 0,
      targetSystem = systemId,
      targetComponent = remoteId
    )

  private val autopilotVersion =
    Autopilot.AutopilotVersion(
      capabilities = Set(
        MAVProtocolCapability.SetAttitudeTarget,
        MAVProtocolCapability.SetPositionTargetLocalNED,
        MAVProtocolCapability.SetPositionTargetGlobalInt,
        MAVProtocolCapability.SetActuatorTarget,
        MAVProtocolCapability.FlightTermination,
        MAVProtocolCapability.CompassCalibration,
        MAVProtocolCapability.MAVLink2,
        MAVProtocolCapability.FlightInformation
      ),
      flightSWVersion = 1,
      middlewareSWVersion = 1,
      oSSWVersion = 1,
      boardVersion = 1,
      flightCustomVersion = IndexedSeq(0, 0, 0, 0, 0, 0, 0, 0),
      middlewareCustomVersion = IndexedSeq(0, 0, 0, 0, 0, 0, 0, 0),
      oSCustomVersion = IndexedSeq(0, 0, 0, 0, 0, 0, 0, 0),
      vendorId = 1,
      productId = 1,
      uid = 0,
      uid2 = IndexedSeq(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
    )

  val read: PartialFunction[MAVMessage, Unit] = {
    case `requestAutopilotCapabilities` =>
      logger.info("GroundControl> READ RequestAutopilotCapabilities")
      logger.info("GroundControl> WRITE CommandAck")
      write(commandAck)
      logger.info("GroundControl> WRITE AutopilotVersion")
      write(autopilotVersion)
  }
}
