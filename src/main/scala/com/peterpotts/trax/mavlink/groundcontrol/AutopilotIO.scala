package com.peterpotts.trax.mavlink.groundcontrol

import com.peterpotts.trax.mavlink.io.{MAVLinkIO, MAVLinkReader}
import com.peterpotts.trax.mavlink.model.MAVMessage

import scala.concurrent.duration._

/**
  * @author Peter Potts
  */
object AutopilotIO extends MAVLinkIO {
  val reconnectInterval: FiniteDuration = 5.second
  val tickInterval: Duration = 1.second

  def on(write: MAVMessage => Unit): MAVLinkReader = AutopilotReader(write)
}
