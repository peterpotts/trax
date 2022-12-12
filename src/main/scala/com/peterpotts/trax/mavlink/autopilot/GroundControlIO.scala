package com.peterpotts.trax.mavlink.autopilot

import com.peterpotts.trax.mavlink.io.{MAVLinkIO, MAVLinkReader}
import com.peterpotts.trax.mavlink.model.MAVMessage

import scala.concurrent.duration._

/**
  * @author Peter Potts
  */
object GroundControlIO extends MAVLinkIO {
  val reconnectInterval: FiniteDuration = 5.second
  val tickInterval: Duration = 1000.milliseconds

  def on(write: MAVMessage => Unit): MAVLinkReader = GroundControlReader(write)
}
