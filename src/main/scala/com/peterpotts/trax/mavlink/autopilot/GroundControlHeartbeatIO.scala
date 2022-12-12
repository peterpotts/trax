package com.peterpotts.trax.mavlink.autopilot

import com.peterpotts.trax.mavlink.model.MAVEnums._
import com.peterpotts.trax.mavlink.model._

/**
  * @author Peter Potts
  */
case class GroundControlHeartbeatIO(write: MAVMessage => Unit) extends AutopilotGroundControl {
  private val remoteHeartbeat =
    GroundControl.Heartbeat(
      `type` = MAVType.GCS,
      autopilot = MAVAutopilot.Invalid,
      baseMode = Set(MAVModeFlag.ManualInputEnabled, MAVModeFlag.SafetyArmed),
      customMode = 0,
      systemStatus = MAVState.Active
    )

  private def localHeartbeat: Autopilot.Heartbeat = {
    Autopilot.Heartbeat(
      `type` = MAVType.Quadrotor,
      autopilot = MAVAutopilot.Generic,
      baseMode =
        if (AutopilotAutomatonState.armed)
          Set(
            MAVModeFlag.ManualInputEnabled,
            MAVModeFlag.StabilizeEnabled,
            MAVModeFlag.GuidedEnabled,
            MAVModeFlag.SafetyArmed
          )
        else
          Set(
            MAVModeFlag.ManualInputEnabled,
            MAVModeFlag.StabilizeEnabled,
            MAVModeFlag.GuidedEnabled
          ),
      customMode = 0,
      systemStatus = if (AutopilotAutomatonState.armed) MAVState.Active else MAVState.Standby
    )
  }

  def tick(sequenceNumber: Int): Unit = {
    logger.trace(s"GroundControl:Tick:$sequenceNumber> WRITE Heartbeat")
    write(localHeartbeat)
  }


  val read: PartialFunction[MAVMessage, Unit] = {
    case `remoteHeartbeat` =>
      logger.trace("GroundControl> READ Heartbeat")
  }
}
