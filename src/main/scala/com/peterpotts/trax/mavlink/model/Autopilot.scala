package com.peterpotts.trax.mavlink.model

import com.peterpotts.trax.mavlink.model.MAVEnums.MAVComponent

/**
  * @author Peter Potts
  */
object Autopilot extends MAVMessages(MAVSchema.SystemId, MAVComponent.IdAutopilot1, MAVSchema.ProtocolVersion)
