package com.peterpotts.trax.mavlink.notes

import com.peterpotts.trax.mavlink.model.Autopilot._
import com.peterpotts.trax.mavlink.model.MAVEnums._

/**
  * @author Peter Potts
  */
object HeartbeatNotes {
  // [tcp:4560:read]
  // <MAVLinkMessage
  // HEARTBEAT seq=1 sysID=1 compID=1 ID=0 CRC=5891
  // type=0
  // autopilot=12
  // base_mode=0
  // custom_mode=0
  // system_status=0
  // mavlink_version=3
  // />

  // PX4 Autopilot has SystemId = 1 and ComponentId = 1
  // PX4 Autopilot sends Heartbeat to the Java simulator.

  val autopilotMessage: Heartbeat =
    Heartbeat(
      `type` = MAVType.Generic,
      autopilot = MAVAutopilot.PX4,
      baseMode = Set.empty,
      customMode = 0,
      systemStatus = MAVState.Uninit
    )

  // [tcp:4560:write]
  // <MAVLinkMessage
  // HEARTBEAT seq=0 sysID=1 compID=51 ID=0 CRC=ffffffff
  // type=0
  // autopilot=0
  // base_mode=0
  // custom_mode=0
  // system_status=0
  // mavlink_version=3
  // />

  // JMAVSim has SystemId = 1 and ComponentId = 51
  // JMAVSim sends Heartbeat to the PX4 Autopilot.

  val simulatorMessage: Heartbeat =
    Heartbeat(
      `type` = MAVType.Generic,
      autopilot = MAVAutopilot.Generic,
      baseMode = Set.empty,
      customMode = 0,
      systemStatus = MAVState.Uninit
    )
}
