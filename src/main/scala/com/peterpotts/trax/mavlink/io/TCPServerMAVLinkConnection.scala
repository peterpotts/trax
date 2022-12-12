package com.peterpotts.trax.mavlink.io

import java.net.{InetSocketAddress, StandardSocketOptions}
import java.nio.channels.spi.AbstractInterruptibleChannel
import java.nio.channels.{ByteChannel, ServerSocketChannel}

/**
  * @author Peter Potts
  */
case class TCPServerMAVLinkConnection(name: String, port: Int) extends ChannelMAVLinkConnection(name) {
  def connect(): AbstractInterruptibleChannel with ByteChannel = {
    val socketAddress = new InetSocketAddress(port)
    prompt("TCP server connecting")
    val serverChannel = ServerSocketChannel.open()
    prompt("TCP client connected")
    serverChannel.socket().bind(socketAddress)
    prompt("TCP client accepting")
    val channel = serverChannel.accept()
    prompt("TCP client accepted")
    channel.configureBlocking(false)
    channel.setOption[java.lang.Boolean](StandardSocketOptions.TCP_NODELAY, true)
    channel
  }
}
