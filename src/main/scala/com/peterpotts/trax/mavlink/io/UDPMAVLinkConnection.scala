package com.peterpotts.trax.mavlink.io

import java.net.InetSocketAddress
import java.nio.channels.spi.AbstractInterruptibleChannel
import java.nio.channels.{ByteChannel, DatagramChannel}

/**
  * @author Peter Potts
  */
case class UDPMAVLinkConnection(
  name: String,
  localPort: Int,
  remoteHostname: String,
  remotePort: Int
) extends ChannelMAVLinkConnection(name) {
  def connect(): AbstractInterruptibleChannel with ByteChannel = {
    val remoteAddress = new InetSocketAddress(remoteHostname, remotePort)
    val localAddress = new InetSocketAddress("0.0.0.0", localPort)
    prompt("UDP opening")
    val channel = DatagramChannel.open()
    prompt("UDP opened")
    channel.socket.bind(localAddress)
    channel.configureBlocking(false)
    channel.connect(remoteAddress)
    channel
  }
}
