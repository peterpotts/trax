package com.peterpotts.trax.mavlink.groundcontrol

import com.peterpotts.trax.mavlink.model.Autopilot._
import com.peterpotts.trax.mavlink.io.MAVLinkReader
import com.peterpotts.trax.mavlink.model.MAVEnums._
import com.peterpotts.trax.mavlink.model.MAVMessage

/**
  * @author Peter Potts
  */
case class AutopilotReader(write: MAVMessage => Unit) extends MAVLinkReader {
  def tick(sequenceNumber: Int): Unit =
    write(Heartbeat(
      `type` = MAVType.GCS,
      autopilot = MAVAutopilot.Invalid,
      baseMode = Set.empty,
      customMode = 0,
      systemStatus = MAVState.Uninit
    ))

  def read(message: MAVMessage): Unit =
    logger.info(s"Autopilot> READ *** $message ***")
}
