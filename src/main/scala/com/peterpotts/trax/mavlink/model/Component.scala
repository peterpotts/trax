package com.peterpotts.trax.mavlink.model

import com.peterpotts.trax.mavlink.model.MAVEnums.MAVComponent
import me.drton.jmavlib.mavlink.MAVLinkMessage

/**
  * @author Peter Potts
  */
object Component {
  private val mapping: Map[Int, MAVMessages] = {
    val components = List(Autopilot, Vehicle, GroundControl)
    components.map(component => component.componentId.value -> component).toMap
  }

  def apply(message: MAVLinkMessage): MAVMessages = mapping.getOrElse(message.componentID, get(message))

  private def get(message: MAVLinkMessage): MAVMessages =
    MAVMessages(message.systemID, MAVComponent(message.componentID), message.protocolVersion)
}
