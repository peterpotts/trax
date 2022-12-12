package com.peterpotts.trax.mavlink.model

import com.peterpotts.trax.mavlink.model.MAVEnums.MAVComponent

/**
  * @author Peter Potts
  */
object GroundControl extends MAVMessages(MAVSchema.SystemId, MAVComponent.IdMissionPlanner, MAVSchema.ProtocolVersion)
