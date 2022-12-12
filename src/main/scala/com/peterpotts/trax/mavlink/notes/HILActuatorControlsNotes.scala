package com.peterpotts.trax.mavlink.notes

import com.peterpotts.trax.mavlink.model.Autopilot._
import com.peterpotts.trax.mavlink.model.MAVEnums.MAVMode

/**
  * @author Peter Potts
  */
object HILActuatorControlsNotes {
  // [tcp:4560:read]
  // <MAVLinkMessage
  // HIL_ACTUATOR_CONTROLS seq=2 sysID=1 compID=1 ID=93 CRC=ec78
  // time_usec=1600641083824000
  // flags=1
  // controls=[Ljava.lang.Object;@51d133e
  // mode=1
  // />

  // PX4 Autopilot has SystemId = 1 and ComponentId = 1
  // PX4 Autopilot sends HILActuatorControls to the Java simulator.

  val autopilotMessage: HILActuatorControls =
    HILActuatorControls(
      timeUSec = 1600641083824000L,
      controls = IndexedSeq(0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f),
      mode = MAVMode.StabilizeDisarmed,
      flags = 1L
    )
}
