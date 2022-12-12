package com.peterpotts.trax.mavlink.groundcontrol

import akka.actor.typed.ActorSystem
import com.peterpotts.trax.mavlink.io.{ConnectionActor, ControlActor, UDPMAVLinkConnection}

/**
  * This application simulates a ground controller.
  * It communicates with an autopilot such as px4_sitl.
  * The autopilot is in turn communicating with a vehicle such as jmavsim.
  *
  * @author Peter Potts
  */
object GroundControlApplication {
  def main(args: Array[String]): Unit = {
    println("git clone https://github.com/PX4/Firmware.git")
    println("cd Firmware")
    println("make px4_sitl jmavsim")

    val autopilot = {
      val connection = UDPMAVLinkConnection("Autopilot", 14550, "localhost", 18570)
      ConnectionActor(connection, AutopilotIO)
    }

    val groundControl = ControlActor("GroundControl", IndexedSeq(autopilot))
    val actorSystem = ActorSystem(groundControl.behavior, "actor-system")
    actorSystem ! ControlActor.Open

  }
}
