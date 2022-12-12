package com.peterpotts.trax.fx

import akka.actor._
import com.peterpotts.trax.models._
import com.peterpotts.trax.usb.ManualController
import scalafx.application.JFXApp
import scalafx.application.JFXApp.PrimaryStage

import scala.concurrent.ExecutionContext
import scala.concurrent.duration._

/**
  * @author Peter Potts
  */
class ModelApplication(model: Model3D) extends JFXApp {
  private val actorSystem = ActorSystem.create("ModelApplication")
  sys.addShutdownHook(actorSystem.terminate())
  private implicit val executionContext: ExecutionContext = actorSystem.dispatcher
  private val scheduler: Scheduler = actorSystem.scheduler
  private val controller = ManualController
  private val world = new World
  private val viewpoint = new Viewpoint
  private val window = new Window(world, viewpoint, controller)
  viewpoint.scene = window

  stage = new PrimaryStage {
    title = model.title
    scene = window
  }

  private val duration = 25.milliseconds
  private var index = 0
  private val steps = model.steps
  private val size = steps.size

  scheduler.scheduleAtFixedRate(duration, duration) {
    new Runnable {
      def run(): Unit = {
        val step = steps(index)
        index = (index + 1) % size
        world.setRotate(model.toEulerAngles(step))
      }
    }
  }
}
