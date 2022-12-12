package com.peterpotts.trax.mavlink.io

import akka.actor.typed.Behavior
import akka.actor.typed.scaladsl.Behaviors
import com.peterpotts.trax.mavlink.io.ControlActor._

import scala.util.control.NonFatal

/**
  * @author Peter Potts
  */
object ControlActor {

  sealed trait Message

  case object Open extends Message

  case object Close extends Message

}

case class ControlActor(name: String, connectionActors: IndexedSeq[ConnectionActor])
  extends PromptLogging(name + ".ControlActor") {

  val behavior: Behavior[Message] = Behaviors.setup { context =>
    val actorRefs =
      for (connectionActor <- connectionActors) yield {
        prompt(s"Spawn ${connectionActor.prompt}")
        context.spawn(connectionActor.behavior, connectionActor.prompt)
      }

    def closedBehavior: Behavior[Message] = Behaviors.receiveMessage {
      case Open =>
        prompt("Open")

        try {
          for (actorRef <- actorRefs) {
            actorRef ! ConnectionActor.Open
          }
        } catch {
          case NonFatal(exception) =>
            // TODO
            exception.printStackTrace()
        }
        openedBehavior
      case Close =>
        prompt("Already closed")
        Behaviors.same
    }

    def openedBehavior: Behavior[Message] = Behaviors.receiveMessage {
      case Open =>
        prompt("Already open")
        Behaviors.same
      case Close =>
        prompt("Close")
        for (actor <- actorRefs) actor ! ConnectionActor.Close
        closedBehavior
    }

    prompt("STARTED")
    closedBehavior
  }
}
