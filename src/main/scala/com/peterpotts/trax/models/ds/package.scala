package com.peterpotts.trax.models

import com.peterpotts.trax.kalman._
import com.peterpotts.trax.math._

package object ds {

  implicit class XDS(val x: X) extends AnyVal {
    def L: Double = x(0)

    def theta: Double = x(1)

    def a: Double = x(2)

    def b: Double = x(3)

    def c: Double = x(4)

    def d: Double = x(5)
  }

  object XDS {
    def apply(
      L: Double,
      theta: Double,
      a: Double,
      b: Double,
      c: Double,
      d: Double
    ): X = X(VectorD(L, theta, a, b, c, d))

    def row(
      L: Vector1 = Vector1.zero,
      theta: Vector1 = Vector1.zero,
      a: Vector1 = Vector1.zero,
      b: Vector1 = Vector1.zero,
      c: Vector1 = Vector1.zero,
      d: Vector1 = Vector1.zero
    ): MatrixD = MatrixD.horizontal(L, theta, a, b, c, d)

    val n: Int = 6
  }

  implicit class UDS(val u: U) extends AnyVal {
    def dummy: Double = u(0)
  }

  object UDS {
    def apply(dummy: Double): U = U(VectorD(dummy))

    val n: Int = 1
  }

  implicit class ZDS(val z: Z) extends AnyVal {
    def lif: Double = z(0)

    def rot: Double = z(1)
  }

  object ZDS {
    def apply(
      lif: Double,
      rot: Double
    ): Z = Z(VectorD(lif, rot))

    def columns(lif: MatrixD, rot: MatrixD): MatrixD = MatrixD.vertical(lif, rot)

    val n: Int = 2
  }

}
