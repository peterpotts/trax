package com.peterpotts.trax.mavlink.io

import com.peterpotts.trax.common.Logging
import com.peterpotts.trax.mavlink.model.MAVEnums.MAVComponent
import com.peterpotts.trax.mavlink.model.{MAVMessage, MAVSchema}

/**
  * @author Peter Potts
  */
trait IO extends Logging {
  val systemId: Int = MAVSchema.SystemId
  val broadcastId: Int = MAVComponent.IdAll.value
  val read: PartialFunction[MAVMessage, Unit]
  val write: MAVMessage => Unit
}
