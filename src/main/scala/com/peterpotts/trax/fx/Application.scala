package com.peterpotts.trax.fx

import akka.actor.{ActorSystem, Scheduler}
import com.peterpotts.trax.usb.ManualController
import scalafx.application.JFXApp
import scalafx.application.JFXApp.PrimaryStage

import scala.concurrent.ExecutionContext
import scala.concurrent.duration._

/**
  * @author Peter Potts
  */
object Application extends JFXApp {
  private val actorSystem = ActorSystem.create("Application")
  sys.addShutdownHook(actorSystem.terminate())
  private implicit val executionContext: ExecutionContext = actorSystem.dispatcher
  private val scheduler: Scheduler = actorSystem.scheduler
  private val controller = ManualController
  private val world = new World
  private val viewpoint = new Viewpoint
  private val window = new Window(world, viewpoint, controller)
  viewpoint.scene = window

  stage = new PrimaryStage {
    title = "Trax"
    scene = window
  }

  private val duration = 100.milliseconds

  scheduler.scheduleAtFixedRate(duration, duration) {
    new Runnable {
      def run(): Unit = {
        controller.poll()
        world.onRotate(controller.eulerAngles)
      }
    }
  }
}
