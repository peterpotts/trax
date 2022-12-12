package com.peterpotts.trax.mavlink.autopilot

import com.peterpotts.trax.common.TimeUSec
import com.peterpotts.trax.mavlink.model._

/**
  * @author Peter Potts
  */
case class GroundControlTimeIO(write: MAVMessage => Unit) extends AutopilotGroundControl {
  val read: PartialFunction[MAVMessage, Unit] = {
    case systemTime: GroundControl.SystemTime =>
      val now = TimeUSec.now()
      val flightTimeUSec = now - systemTime.timeUnixUSec
      val bootTimeMS = systemTime.timeBootMS
      logger.info(s"GroundControl> READ SystemTime [flight time $flightTimeUSec us / boot time $bootTimeMS ms]")
  }
}
