package com.peterpotts.trax.mavlink.autopilot

import com.peterpotts.trax.mavlink.io.IO
import com.peterpotts.trax.mavlink.model.MAVEnums._

/**
  * @author Peter Potts
  */
trait AutopilotVehicleControl extends IO {
  val localId: Int = MAVComponent.IdAutopilot1.value
  val remoteId: Int = MAVComponent.IdMissionPlanner.value
}
