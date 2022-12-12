package com.peterpotts.trax.mavlink.autopilot

import com.peterpotts.trax.mavlink.io.IO
import com.peterpotts.trax.mavlink.model.MAVEnums.MAVComponent

/**
  * @author Peter Potts
  */
trait AutopilotGroundControl extends IO {
  val localId: Int = MAVComponent.IdAutopilot1.value
  val remoteId: Int = MAVComponent.IdMissionPlanner.value
}
