package com.peterpotts.trax.mavlink.autopilot

import com.peterpotts.trax.mavlink.model.GroundControl

/**
  * {{{
  * Pitch takes value from -1000 to 1000.
  * Roll takes value from -1000 to 1000.
  * Yaw takes value from -1000 to 1000.
  * Throttle takes value from 0 to 1000.
  * }}}
  *
  * @author Peter Potts
  */
object ManualControlDecorator {

  implicit class DecoratedManualControl(val manualControl: GroundControl.ManualControl) extends AnyVal {
    def pitch: Int = manualControl.x

    def roll: Int = manualControl.y

    def yaw: Int = manualControl.r

    def throttle: Int = manualControl.z

    def button(index: Int): Boolean = ((manualControl.buttons >> index) & 1) == 1

    def display: String =
      Seq(
        "Pitch",
        pitch,
        "Roll",
        roll,
        "Yaw",
        yaw,
        "Throttle",
        throttle,
        "Button 1",
        button(0),
        "Button 2",
        button(1),
        "Button 3",
        button(2),
        "Button 4",
        button(3),
        "Button 5",
        button(4)
      ).mkString(" ")
  }

}
