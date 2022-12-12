package com.peterpotts.trax.models

import com.peterpotts.trax.kalman._
import com.peterpotts.trax.math._

/**
  * @author Peter Potts
  */
package object b {

  implicit class XB(val x: X) extends AnyVal {
    def displacement: Double = x(0)

    def velocity: Double = x(1)

    def acceleration: Double = x(2)
  }

  object XB {
    def apply(displacement: Double, velocity: Double, acceleration: Double): X =
      X(VectorD(displacement, velocity, acceleration))

    val n: Int = 3
  }

  implicit class UB(val u: U) extends AnyVal {
    def force: Double = u(0)
  }

  object UB {
    def apply(force: Double): U = U(VectorD(force))

    val n: Int = 1
  }

  implicit class ZB(val z: Z) extends AnyVal {
    def accelerometer: Double = z(0)
  }

  object ZB {
    def apply(accelerometer: Double): Z = Z(VectorD(accelerometer))

    val n: Int = 1
  }

}
