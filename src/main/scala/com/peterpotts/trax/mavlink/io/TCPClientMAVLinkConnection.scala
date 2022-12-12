package com.peterpotts.trax.mavlink.io

import java.net.InetSocketAddress
import java.nio.channels.spi.AbstractInterruptibleChannel
import java.nio.channels.{ByteChannel, SocketChannel}

/**
  * @author Peter Potts
  */
case class TCPClientMAVLinkConnection(name: String, hostname: String, port: Int)
  extends ChannelMAVLinkConnection(name) {

  def connect(): AbstractInterruptibleChannel with ByteChannel = {
    val socketAddress = new InetSocketAddress(hostname, port)
    prompt("TCP client opening")
    val channel = SocketChannel.open(socketAddress)
    prompt("TCP client opened")
    channel
  }
}
