package com.peterpotts.trax.mavlink.io

import akka.actor.typed.scaladsl.Behaviors
import akka.actor.typed.{ActorRef, Behavior}
import com.peterpotts.trax.mavlink.io.InputActor._
import com.peterpotts.trax.mavlink.model.MAVDecoder
import me.drton.jmavlib.mavlink.MAVLinkMessage

/**
  * @author Peter Potts
  */
object InputActor {

  sealed trait Message

  case class Connect(from: ActorRef[ConnectionActor.Message]) extends Message

  case object Read extends Message

}

case class InputActor(connection: MAVLinkConnection, io: MAVLinkIO, write: MAVLinkMessage => Unit)
  extends PromptLogging(connection.name + ".InputActor") {
  private val reader = io.on(write)

  val behavior: Behavior[Message] = Behaviors.setup { context =>
    def disconnectedBehavior: Behavior[Message] = Behaviors.receiveMessage {
      case connect: Connect =>
        prompt("Connecting")
        context.self ! Read
        connectedBehavior(connect)
      case Read =>
        prompt("Drop read")
        Behaviors.same
    }

    def connectedBehavior(connect: Connect): Behavior[Message] = Behaviors.receiveMessage {
      case _: Connect =>
        prompt("Already connected")
        Behaviors.same
      case Read =>
        try {
          val message = MAVDecoder(connection.read())
          reader.read(message)
          context.self ! Read
          Behaviors.same
        } catch {
          case exception: Exception =>
            exception.printStackTrace()
            connect.from ! ConnectionActor.Close
            prompt("Disconnecting")
            disconnectedBehavior
        }
    }

    prompt("STARTED")
    disconnectedBehavior
  }
}
