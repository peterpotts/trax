package com.peterpotts.trax.mavlink.notes

import com.peterpotts.trax.mavlink.model.Autopilot._

/**
  * @author Peter Potts
  */
object SystemTimeNotes {
  // [tcp:4560:write]
  // <MAVLinkMessage SYSTEM_TIME seq=0 sysID=1 compID=51 ID=2 CRC=ffffffff
  // time_unix_usec=1600641084042000
  // time_boot_ms=2913249596
  // />

  // JMAVSim has SystemId = 1 and ComponentId = 51
  // JMAVSim sends SystemTime to PX4 Autopilot.

  val simulatorMessage: SystemTime =
    SystemTime(
      timeUnixUSec = 1600641084042000L, // Unix time in microseconds
      timeBootMS = -1381717700 // Time since boot in milliseconds
    )
}
