package com.peterpotts.trax.mavlink.autopilot

import com.peterpotts.trax.common.TimeUSec
import com.peterpotts.trax.mavlink.model._

/**
  * @author Peter Potts
  */
case class GroundControlPingIO(write: MAVMessage => Unit) extends AutopilotGroundControl {
  private def localPing(seq: Int) =
    Autopilot.Ping(
      timeUSec = TimeUSec.now(),
      seq = seq,
      targetSystem = systemId,
      targetComponent = remoteId
    )

  def tick(sequenceNumber: Int): Unit = {
    logger.trace(s"GroundControl:Tick:$sequenceNumber> WRITE Ping")
    write(localPing(sequenceNumber))
  }

  val read: PartialFunction[MAVMessage, Unit] = {
    case ping: GroundControl.Ping =>
      val now = TimeUSec.now()
      val flightTimeUSec = now - ping.timeUSec
      logger.trace(s"GroundControl> READ Ping [flight time $flightTimeUSec us]")
  }
}
