package com.peterpotts.trax.mavlink.io

import akka.actor.typed.Behavior
import akka.actor.typed.scaladsl.Behaviors
import com.peterpotts.trax.mavlink.io.ConnectionActor._

/**
  * @author Peter Potts
  */
object ConnectionActor {

  sealed trait Message

  case object Open extends Message

  case object Close extends Message

}

case class ConnectionActor(connection: MAVLinkConnection, io: MAVLinkIO)
  extends PromptLogging(connection.name + ".ConnectionActor") {

  val behavior: Behavior[Message] = Behaviors.setup { context =>
    val outputActor = OutputActor(connection, io)
    prompt(s"Spawn ${outputActor.prompt}")
    val outputActorRef = context.spawn(outputActor.behavior, outputActor.prompt)
    val write = OutputActor.writer(outputActorRef)(_)
    val inputActor = InputActor(connection, io, write)
    prompt(s"Spawn ${inputActor.prompt}")
    val inputActorRef = context.spawn(inputActor.behavior, inputActor.prompt)

    def closedBehavior: Behavior[Message] = Behaviors.receiveMessage {
      case Open =>
        prompt("Opening")

        try {
          connection.open()
          inputActorRef ! InputActor.Connect(context.self)
          outputActorRef ! OutputActor.Connect(context.self)
          openedBehavior(connection)
        } catch {
          case _: Exception =>
            prompt("Schedule open")
            context.scheduleOnce(io.reconnectInterval, context.self, Open)
            Behaviors.same
        }
      case Close =>
        prompt("Already closed")
        Behaviors.same
    }

    def openedBehavior(connection: MAVLinkConnection): Behavior[Message] = Behaviors.receiveMessage {
      case Open =>
        prompt("Already open")
        Behaviors.same
      case Close =>
        prompt("Closing")

        try connection.close() catch {
          case _: Exception =>
        }

        prompt("Schedule open")
        context.scheduleOnce(io.reconnectInterval, context.self, Open)
        closedBehavior
    }

    closedBehavior
  }
}
