package com.peterpotts.trax.mavlink.io

import me.drton.jmavlib.mavlink.MAVLinkMessage

/**
  * @author Peter Potts
  */
trait MAVLinkConnection {
  val name: String

  def open(): Unit

  def close(): Unit

  def write(message: MAVLinkMessage): Unit

  def read(): MAVLinkMessage
}
