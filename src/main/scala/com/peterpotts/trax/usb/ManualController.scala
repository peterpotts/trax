package com.peterpotts.trax.usb

import com.peterpotts.trax.common.Logging
import com.peterpotts.trax.math.Vector3
import net.java.games.input.Component.Identifier._
import net.java.games.input._

/**
  * @author Peter Potts
  */
object ManualController extends ManualController("I-Controller")

class ManualController(name: String) extends Logging {
  private var zeros: Map[Component.Identifier, Float] = Map.empty
  private var values: Map[Component.Identifier, Float] = Map.empty

  private def getFloat(key: Component.Identifier): Float = values.getOrElse(key, 0F) - zeros.getOrElse(key, 0F)

  private def toBoolean(value: Float): Boolean = value == 1F

  private def getBoolean(key: Component.Identifier): Boolean = toBoolean(values.getOrElse(key, 0F))

  def roll: Float = getFloat(Component.Identifier.Axis.X)

  def pitch: Float = getFloat(Component.Identifier.Axis.Y)

  def throttle: Float = -getFloat(Component.Identifier.Axis.Z)

  def yaw: Float = getFloat(Component.Identifier.Axis.RY)

  def eulerAngles: Vector3 = Vector3(roll, pitch, yaw)

  def button0: Boolean = getBoolean(Component.Identifier.Button._0)

  def button1: Boolean = getBoolean(Component.Identifier.Button._1)

  def button3: Boolean = getBoolean(Component.Identifier.Button._3)

  def button4: Boolean = getBoolean(Component.Identifier.Button._4)

  private lazy val maybeController = ControllerEnvironment.getDefaultEnvironment.getControllers.find(_.getName == name)
  private val event = new Event

  def poll(): Unit =
    maybeController.foreach { controller =>
      if (controller.poll()) {
        val queue = controller.getEventQueue

        while (queue.getNextEvent(event)) {
          val component = event.getComponent
          val key = component.getIdentifier
          val value = component.getPollData
          if (!values.contains(key)) zeros = zeros + (key -> value)
          values = values + (key -> value)

          key match {
            case button: Button => logger.info(s"Button ${button.getName} = ${toBoolean(value)}")
            case key: Key => logger.info(s"Key ${key.getName} = ${toBoolean(value)}")
            case _: Axis =>
          }
        }
      }
    }

  def reset(): Unit = {
    zeros = values
    logger.info("Reset zero values")
  }
}
