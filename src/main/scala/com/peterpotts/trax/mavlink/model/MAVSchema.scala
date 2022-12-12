package com.peterpotts.trax.mavlink.model

import me.drton.jmavlib.mavlink.MAVLinkSchema

/**
  * @author Peter Potts
  */
object MAVSchema extends MAVSchema("src/main/resources/message/common.xml") {
  val SystemId = 1
  val ProtocolVersion = 1
}

case class MAVSchema(xmlFileName: String) extends MAVLinkSchema(xmlFileName)
