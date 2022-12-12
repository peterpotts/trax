package com.peterpotts.trax.mavlink.model

import com.peterpotts.trax.mavlink.model.MAVEnums.MAVComponent
import me.drton.jmavlib.mavlink.MAVLinkMessage

/**
  * @author Peter Potts
  */
abstract class MAVMessage(name: String, systemId: Int, componentId: MAVComponent, protocolVersion: Int)
  extends MAVLinkMessage(MAVSchema, name, systemId, componentId.value, protocolVersion)
