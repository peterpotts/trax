package com.peterpotts.trax.mavlink.autopilot

import com.peterpotts.trax.mavlink.model.Vehicle

/**
  * @author Peter Potts
  */
object HILStateQuaternionDecorator {

  implicit class DecoratedHILStateQuaternion(val hILStateQuaternion: Vehicle.HILStateQuaternion) extends AnyVal {
    def qW: Float = hILStateQuaternion.attitudeQuaternion(0)

    def qX: Float = hILStateQuaternion.attitudeQuaternion(1)

    def qY: Float = hILStateQuaternion.attitudeQuaternion(2)

    def qZ: Float = hILStateQuaternion.attitudeQuaternion(3)
  }

}
