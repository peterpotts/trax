package com.peterpotts.trax.mavlink.notes

import com.peterpotts.trax.mavlink.model.Autopilot._

/**
  * @author Peter Potts
  */
object HILStateQuaternionNotes {
  // [tcp:4560:write]
  // <MAVLinkMessage
  // HIL_STATE_QUATERNION seq=0 sysID=1 compID=51 ID=115 CRC=ffffffff
  // time_usec=1600641083708000
  // attitude_quaternion=[Ljava.lang.Object;@6bdd1ecc
  // rollspeed=0.0
  // pitchspeed=0.0
  // yawspeed=0.0
  // lat=473977420
  // lon=85455940
  // alt=0
  // vx=0
  // vy=0
  // vz=0
  // ind_airspeed=0
  // true_airspeed=0
  // xacc=0
  // yacc=0
  // zacc=0
  // />

  // JMAVSim has SystemId = 1 and ComponentId = 51
  // JMAVSim sends HILStateQuaternion to PX4 Autopilot.

  val simulatorMessage: HILStateQuaternion =
    HILStateQuaternion(
      timeUSec = 1600641083708000L, // Unix time in microseconds
      attitudeQuaternion = IndexedSeq(1f, 0f, 0f, 0f), // null rotation
      rollSpeed = 0f, // rad/s
      pitchSpeed = 0f, // rad/s
      yawSpeed = 0f, // rad/s
      lat = 473977420, // 47.3977420 N
      lon = 85455940, // 8.5455940 E
      alt = 0, // mm
      vX = 0, // cm/s
      vY = 0, // cm/s
      vZ = 0, // cm/s
      indAirspeed = 0, // cm/s
      trueAirspeed = 0, // cm/s
      xAcc = 0, // mg
      yAcc = 0, // mg
      zAcc = 0 // mg
    )
}
