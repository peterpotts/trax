package com.peterpotts.trax.models

import com.peterpotts.trax.kalman._
import com.peterpotts.trax.math.VectorD

/**
  * @author Peter Potts
  */
package object a {
  implicit class XA(val x: X) extends AnyVal {
    def displacement: Double = x(0)

    def velocity: Double = x(1)
  }

  object XA {
    def apply(displacement: Double, velocity: Double): X = X(VectorD(displacement, velocity))

    val n: Int = 2
  }

  implicit class UA(val u: U) extends AnyVal {
    def force: Double = u(0)
  }

  object UA {
    def apply(force: Double): U = U(VectorD(force))

    val n: Int = 1
  }

  implicit class ZA(val z: Z) extends AnyVal {
    def displacement: Double = z(0)
  }

  object ZA {
    def apply(displacement: Double): Z = Z(VectorD(displacement))

    val n: Int = 1
  }

}
