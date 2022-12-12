package com.peterpotts.trax.mavlink.io

import com.peterpotts.trax.common.Logging
import com.peterpotts.trax.mavlink.model.MAVMessage

/**
  * @author Peter Potts
  */
trait MAVLinkReader extends Logging {
  def tick(sequenceNumber: Int): Unit

  def read(message: MAVMessage): Unit
}
