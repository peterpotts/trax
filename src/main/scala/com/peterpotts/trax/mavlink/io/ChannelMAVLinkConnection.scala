package com.peterpotts.trax.mavlink.io

import java.io.IOException
import java.nio.channels.ByteChannel
import java.nio.channels.spi.AbstractInterruptibleChannel

import com.peterpotts.trax.mavlink.model.MAVSchema
import com.peterpotts.trax.mavlink.io
import me.drton.jmavlib.mavlink.{MAVLinkMessage, MAVLinkStream}

/**
  * @author Peter Potts
  */
abstract class ChannelMAVLinkConnection(name: String) extends PromptLogging(name) with MAVLinkConnection {
  private var maybeChannelStream: Option[ChannelStream] = None

  def connect(): AbstractInterruptibleChannel with ByteChannel

  def open(): Unit =
    if (maybeChannelStream.isEmpty) {
      prompt("Channel opening")
      prompt("Channel connecting")
      val channel = connect()
      prompt("Channel connected")
      val stream = new MAVLinkStream(MAVSchema, channel)
      stream.setDebug(true)
      io.ChannelStream(channel, stream)
      prompt("Channel opened")
      maybeChannelStream = Some(io.ChannelStream(channel, stream))
    } //else {
      //throw new IOException("Already open")
    //}

  def close(): Unit =
    maybeChannelStream.foreach { channelStream =>
      channelStream.close()
      maybeChannelStream = None
    }

  def write(message: MAVLinkMessage): Unit = maybeChannelStream.fold(throw new IOException("Closed"))(_.write(message))

  def read(): MAVLinkMessage = maybeChannelStream.fold(throw new IOException("Closed"))(_.read())
}
