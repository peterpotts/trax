package com.peterpotts.trax.mavlink.autopilot

import com.peterpotts.trax.mavlink.io.MAVLinkReader
import com.peterpotts.trax.mavlink.model.MAVMessage

/**
  * @author Peter Potts
  */
case class VehicleReader(write: MAVMessage => Unit) extends MAVLinkReader {
  private val heartbeatIO = VehicleHeartbeatIO(write)
  private val stateIO = VehicleStateIO(write)
  private val controlsIO = VehicleControlIO(write)

  private val default: PartialFunction[MAVMessage, Unit] = {
    case message => logger.info(s"Vehicle> READ *** $message ***")
  }

  private val partialFunction =
    heartbeatIO.read orElse
      stateIO.read orElse
      controlsIO.read orElse
      default

  def tick(sequenceNumber: Int): Unit = {
    heartbeatIO.tick(sequenceNumber)
    controlsIO.tick(sequenceNumber)
  }

  def read(message: MAVMessage): Unit = partialFunction(message)
}
