package com.peterpotts.trax.mavlink.model

import com.peterpotts.trax.mavlink.model.MAVEnums.MAVComponent

/**
  * @author Peter Potts
  */
object Vehicle extends MAVMessages(MAVSchema.SystemId, new MAVComponent(51), MAVSchema.ProtocolVersion)
