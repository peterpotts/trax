package com.peterpotts.trax.mavlink.notes

import com.peterpotts.trax.mavlink.model.Autopilot._
import com.peterpotts.trax.mavlink.model.MAVEnums._

/**
  * @author Peter Potts
  */
object CommandLongNotes {
  // [tcp:4560:read]
  // <MAVLinkMessage
  // COMMAND_LONG seq=0 sysID=1 compID=1 ID=76 CRC=0419
  // target_system=0
  // target_component=0
  // command=511
  // confirmation=0
  // param1=115.0
  // param2=5000.0
  // param3=0.0
  // param4=0.0
  // param5=0.0
  // param6=0.0
  // param7=0.0
  // />

  // PX4 Autopilot has SystemId = 1 and ComponentId = 1
  // PX4 Autopilot requests a HILStateQuaternion message every 0.005 seconds from the Java simulator.

  val autopilotMessage: CommandLong =
    CommandLong(
      targetSystem = 0,
      targetComponent = 0,
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
}
