package com.peterpotts.trax.mavlink.autopilot

import akka.actor.typed.ActorSystem
import com.peterpotts.trax.mavlink.io._

/** ConnectionActor
  *
  * @author Peter Potts
  */

object AutopilotApplication {
  def main(args: Array[String]): Unit = {
    println("run QGroundControl")
    println("git clone https://github.com/PX4/Firmware.git")
    println("cd Firmware")
    println("make")
    println("cd Tools")
    println("source jmavsim_run.sh")
    println("run vehicle")
    println("press F1 for help")

    val vehicle = {
      val connection = TCPClientMAVLinkConnection("Vehicle", "localhost", 4560)
      ConnectionActor(connection, VehicleIO)
    }

    val groundControl = {
      val connection = UDPMAVLinkConnection("GroundControl", 18570, "localhost", 14550)
      ConnectionActor(connection, GroundControlIO)
    }

    val autopilot = ControlActor("Autopilot", IndexedSeq(vehicle, groundControl))
    val actorSystem = ActorSystem(autopilot.behavior, "actor-system")
    actorSystem ! ControlActor.Open
  }
}
