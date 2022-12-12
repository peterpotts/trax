package com.peterpotts.trax.mavlink.io

import java.nio.channels.spi.AbstractInterruptibleChannel

import me.drton.jmavlib.mavlink.{MAVLinkMessage, MAVLinkStream}

import scala.util.Try

/**
  * @author Peter Potts
  */
case class ChannelStream(channel: AbstractInterruptibleChannel, stream: MAVLinkStream) {
  def close(): Unit = Try(channel.close())

  def write(message: MAVLinkMessage): Unit = {
    // println(s"[Write] msg.name: ${message.getMsgName}, type: ${message.getMsgType}")
    stream.write(message)
  }

  def read(): MAVLinkMessage = {

    val message = {
      var message = stream.read()

      while (message == null) {
        Thread.sleep(1L)
        message = stream.read()
      }

      message
    }

    // println(s"[Read] msg.name: ${message.getMsgName}, type: ${message.getMsgType}")
    message
  }
}
