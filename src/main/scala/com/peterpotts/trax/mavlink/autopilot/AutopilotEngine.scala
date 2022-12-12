package com.peterpotts.trax.mavlink.autopilot

/**
  * @author Peter Potts
  */
object AutopilotEngine {

  // GroundControlManualIO: GroundControl.ManualControl
//  def manualControlInputs(joystick: Joystick): Unit = {
//    Joystick.set(joystick)
//
//    if (VehicleState.armed) {
//      val servoRaw = 1000 + joystick.throttle
//      VehicleControl.setServoRawValues(IndexedSeq(servoRaw, servoRaw, servoRaw, servoRaw, 1500, 1500))
//      VehicleControl.resetRSSI()
//    } else {
//      VehicleControl.reset()
//    }
//  }

  // VehicleControlIO: Autopilot.HILActuatorControls
//  def servoInputs(size: Int): IndexedSeq[Float] =
//    IndexedSeq.tabulate(size)(VehicleControl.getControlOutputs)

  // GroundControlStateVectorIO: Autopilot.ServoOutputRaw
//  def servoOutputs(size: Int): IndexedSeq[Int] =
//    IndexedSeq.tabulate(size)(VehicleControl.getServoRawValue)
}
