package com.peterpotts.trax.mavlink.io

import akka.actor.typed.scaladsl.Behaviors
import akka.actor.typed.{ActorRef, Behavior}
import com.peterpotts.trax.mavlink.io.OutputActor._
import me.drton.jmavlib.mavlink.MAVLinkMessage

import scala.concurrent.duration.FiniteDuration

/**
  * @author Peter Potts
  */
object OutputActor {

  sealed trait Message

  case class Connect(from: ActorRef[ConnectionActor.Message]) extends Message

  case object Tick extends Message

  case class Write(message: MAVLinkMessage) extends Message

  def writer(actorRef: ActorRef[Message])(message: MAVLinkMessage): Unit = actorRef ! OutputActor.Write(message)
}

case class OutputActor(connection: MAVLinkConnection, io: MAVLinkIO)
  extends PromptLogging(connection.name + ".OutputActor") {

  val behavior: Behavior[Message] = Behaviors.setup { context =>
    val reader = io.on(writer(context.self))

    def scheduleTick(): Unit =
      io.tickInterval match {
        case finiteDuration: FiniteDuration => context.scheduleOnce(finiteDuration, context.self, Tick)
        case _ => prompt("No schedule keep-alive")
      }

    def disconnectedBehavior: Behavior[Message] = Behaviors.receiveMessage {
      case connect: Connect =>
        prompt("Connecting")
        scheduleTick()
        connectedBehavior(connect)
      case Tick =>
        prompt("Dropping keep-alive")
        Behaviors.same
      case Write(_) =>
        prompt("Dropping message")
        Behaviors.same
    }

    def connectedBehavior(connect: Connect): Behavior[Message] = {
      var sequenceNumber = 0

      Behaviors.receiveMessage {
        case _: Connect =>
          prompt("Already connected")
          Behaviors.same
        case Tick =>
          reader.tick(sequenceNumber)
          sequenceNumber = sequenceNumber + 1
          scheduleTick()
          Behaviors.same
        case Write(message) =>
          try {
            connection.write(message)
            Behaviors.same
          } catch {
            case exception: Exception =>
              exception.printStackTrace()
              connect.from ! ConnectionActor.Close
              prompt("Disconnecting")
              disconnectedBehavior
          }
      }
    }

    prompt("STARTED")
    disconnectedBehavior
  }
}
