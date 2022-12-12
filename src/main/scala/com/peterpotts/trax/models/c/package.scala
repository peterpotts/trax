package com.peterpotts.trax.models

import com.peterpotts.trax.kalman._
import com.peterpotts.trax.math.{Matrix22, MatrixD, Vector2, VectorD}

/**
  * @author Peter Potts
  */
package object c {

  implicit class XC(val x: X) extends AnyVal {
    def d: Vector2 = Vector2(x(0), x(1))

    def v: Vector2 = Vector2(x(2), x(3))

    def -(that: X): X = XC(
      d = d - that.d,
      v = v - that.v
    )
  }

  object XC {
    def apply(d: Vector2, v: Vector2): X = X(VectorD.vertical(d, v))

    def columns(d: MatrixD, v: MatrixD): MatrixD = MatrixD.vertical(d, v)

    def row(
      d: Matrix22 = Matrix22.zero,
      v: Matrix22 = Matrix22.zero
    ): MatrixD = MatrixD.horizontal(d, v)

    val n: Int = 4
  }

  implicit class UC(val u: U) extends AnyVal {
    def F: Vector2 = Vector2(u(0), u(1))
  }

  object UC {
    def apply(F: Vector2): U = U(VectorD.vertical(F))

    def columns(F: MatrixD): MatrixD = MatrixD.vertical(F)

    def row(F: Matrix22 = Matrix22.zero): MatrixD = MatrixD.horizontal(F)

    val n: Int = 2
  }

  implicit class ZC(val z: Z) extends AnyVal {
    def dis: Vector2 = Vector2(z(0), z(1))
  }

  object ZC {
    def apply(dis: Vector2): Z = Z(VectorD.vertical(dis))

    def columns(dis: MatrixD): MatrixD = MatrixD.vertical(dis)

    def row(dis: Matrix22 = Matrix22.zero): MatrixD = MatrixD.horizontal(dis)

    val n: Int = 2
  }

}
