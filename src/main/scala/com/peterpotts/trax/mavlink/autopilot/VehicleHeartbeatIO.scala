package com.peterpotts.trax.mavlink.autopilot

import com.peterpotts.trax.mavlink.model.MAVEnums._
import com.peterpotts.trax.mavlink.model._

/**
  * Read from Vehicle.
  * Write from Autopilot.
  *
  * @author Peter Potts
  */
case class VehicleHeartbeatIO(write: MAVMessage => Unit) extends AutopilotVehicleControl {
  private val localHeartbeat =
    Autopilot.Heartbeat(
      `type` = MAVType.Generic,
      // autopilot = MAVAutopilot.PX4,
      autopilot = MAVAutopilot.Generic,
      baseMode = Set.empty,
      customMode = 0,
      systemStatus = MAVState.Uninit
    )

  private val remoteHeartbeat =
    Vehicle.Heartbeat(
      `type` = MAVType.Generic,
      autopilot = MAVAutopilot.Generic,
      baseMode = Set.empty,
      customMode = 0,
      systemStatus = MAVState.Uninit
    )

  private val setMessageInterval =
    Autopilot.CommandLong(
      targetSystem = systemId,
      targetComponent = broadcastId,
      command = MAVCmd.SetMessageInterval,
      confirmation = 0, // First transmission of this command.
      param1 = 115f, // HILStateQuaternion
      param2 = 5000f, // 0.005 seconds
      param3 = 0f,
      param4 = 0f,
      param5 = 0f,
      param6 = 0f,
      param7 = 0f
    )

  def tick(sequenceNumber: Int): Unit =
    if (!AutopilotAutomatonState.initialized) {
      logger.info(s"Vehicle:Tick:$sequenceNumber> WRITE CommandLong [ignored by jMAVSim]")
      write(setMessageInterval)
      logger.trace(s"Vehicle:Tick:$sequenceNumber> WRITE Heartbeat")
      write(localHeartbeat)
      AutopilotAutomatonState.initialized = true
      AutopilotControlState.VarLandedState.set(MAVLandedState.OnGround)
    }

  val read: PartialFunction[MAVMessage, Unit] = {
    case `remoteHeartbeat` =>
      logger.trace("Vehicle> READ Heartbeat")
  }
}
