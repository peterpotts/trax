package com.peterpotts.trax.mavlink.autopilot

import com.peterpotts.trax.mavlink.autopilot.ManualControlDecorator._
import com.peterpotts.trax.mavlink.model._

/**
  * @author Peter Potts
  */
case class GroundControlManualIO(write: MAVMessage => Unit) extends AutopilotGroundControl {
  val read: PartialFunction[MAVMessage, Unit] = {
    case manualControl: GroundControl.ManualControl if manualControl.target == localId =>
      logger.trace(s"GroundControl> READ ManualControl ${manualControl.display}")
      AutopilotControlState.VarManualControl.set(manualControl)
  }
}
