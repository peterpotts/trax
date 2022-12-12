package com.peterpotts.trax.mavlink.notes

/**
  * @author Peter Potts
  */
object ProtocolNotes {
  // PX4 Autopilot requests a HILStateQuaternion message every 0.005 seconds from the Java simulator.
  // PX4 Autopilot sends Heartbeat to the Java simulator.
  // PX4 Autopilot sends HILActuatorControls to the Java simulator.

  // JMAVSim sends Heartbeat to the PX4 Autopilot.
  // JMAVSim sends HILStateQuaternion to PX4 Autopilot.
  // JMAVSim sends HILSensor to PX4 Autopilot.
  // JMAVSim sends SystemTime to PX4 Autopilot.
}
