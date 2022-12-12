package com.peterpotts.trax.mavlink.io

import com.peterpotts.trax.mavlink.model.MAVMessage

import scala.concurrent.duration.{Duration, FiniteDuration}

/**
  * @author Peter Potts
  */
trait MAVLinkIO {
  val reconnectInterval: FiniteDuration
  val tickInterval: Duration

  def on(write: MAVMessage => Unit): MAVLinkReader
}
