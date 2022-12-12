package com.peterpotts.trax.mavlink.notes

import com.peterpotts.trax.mavlink.model.Autopilot._

/**
  * @author Peter Potts
  */
object HILSensorNotes {
  // [tcp:4560:write]
  // <MAVLinkMessage
  // HIL_SENSOR seq=0 sysID=1 compID=51 ID=107 CRC=ffffffff
  // time_usec=1600641083708000
  // xacc=-0.06816761
  // yacc=-0.0025632957
  // zacc=-9.8282385
  // xgyro=-0.012046869
  // ygyro=0.0031549192
  // zgyro=-0.007994664
  // xmag=0.22679141
  // ymag=0.004107161
  // zmag=0.42397952
  // abs_pressure=955.9743
  // diff_pressure=0.0
  // pressure_alt=488.01447
  // temperature=0.0
  // fields_updated=7167
  // />

  // JMAVSim has SystemId = 1 and ComponentId = 51
  // JMAVSim sends HILSensor to PX4 Autopilot.

  val simulatorMessage: HILSensor =
    HILSensor(
      timeUSec = 1600641083708000L, // Unix time in microseconds
      xAcc = -0.06816761f, // m/s^^2 - bit 0
      yAcc = -0.0025632957f, // m/s^^2 - bit 1
      zAcc = -9.8282385f, // m/s^^2 - bit 2
      xGyro = -0.012046869f, // rad/s - bit 3
      yGyro = 0.0031549192f, // rad/s - bit 4
      zGyro = -0.007994664f, // rad/s - bit 5
      xMag = 0.22679141f, // Gauss - bit 6
      yMag = 0.004107161f, // Gauss - bit 7
      zMag = 0.42397952f, // Gauss - bit 8
      absPressure = 955.9743f, // millibar - bit 9
      diffPressure = 0f, // (airspeed) millibar - bit 10
      pressureAlt = 488.01447f, // m - bit 11
      temperature = 0f, // celsius - bit 12
      fieldsUpdated = 7167 // Binary 1101111111111 So all updated except diffPressure
    )
}
