package com.peterpotts.trax.models

import com.peterpotts.trax.kalman._
import com.peterpotts.trax.math._

/**
  * @author Peter Potts
  */
package object f {

  implicit class XF(val x: X) extends AnyVal {
    def d: Vector2 = Vector2(x(0), x(1))

    def v: Vector2 = Vector2(x(2), x(3))

    def a: Vector2 = Vector2(x(4), x(5))

    def theta: Angle = Angle.normalize(x(6))

    def omega: Vector1 = Vector1(x(7))

    def alpha: Vector1 = Vector1(x(8))

    def magOffB: Vector2 = Vector2(x(9), x(10))

    def gyrOff: Vector1 = Vector1(x(11))

    def m: Vector1 = Vector1(x(12))

    def -(that: X): X = XF(
      d = d - that.d,
      v = v - that.v,
      a = a - that.a,
      theta = theta - that.theta,
      omega = omega - that.omega,
      alpha = alpha - that.alpha,
      magOffB = magOffB - that.magOffB,
      gyrOff = gyrOff - that.gyrOff,
      m = m - that.m
    )
  }

  object XF {
    def apply(
      d: Vector2,
      v: Vector2,
      a: Vector2,
      theta: Angle,
      omega: Vector1,
      alpha: Vector1,
      magOffB: Vector2,
      gyrOff: Vector1,
      m: Vector1
    ): X = X(VectorD.vertical(d, v, a, theta.vector1, omega, alpha, magOffB, gyrOff, m))

    def columns(
      d: MatrixD,
      v: MatrixD,
      a: MatrixD,
      theta: MatrixD,
      omega: MatrixD,
      alpha: MatrixD,
      magOffB: MatrixD,
      gyrOff: MatrixD,
      m: MatrixD
    ): MatrixD = MatrixD.vertical(d, v, a, theta, omega, alpha, magOffB, gyrOff, m)

    def oneRow(
      d: Matrix12 = Matrix12.zero,
      v: Matrix12 = Matrix12.zero,
      a: Matrix12 = Matrix12.zero,
      theta: Vector1 = Vector1.zero,
      omega: Vector1 = Vector1.zero,
      alpha: Vector1 = Vector1.zero,
      magOffB: Matrix12 = Matrix12.zero,
      gyrOff: Vector1 = Vector1.zero,
      m: Vector1 = Vector1.zero
    ): MatrixD = MatrixD.horizontal(d, v, a, theta, omega, alpha, magOffB, gyrOff, m)

    def twoRows(
      d: Matrix22 = Matrix22.zero,
      v: Matrix22 = Matrix22.zero,
      a: Matrix22 = Matrix22.zero,
      theta: Vector2 = Vector2.zero,
      omega: Vector2 = Vector2.zero,
      alpha: Vector2 = Vector2.zero,
      magOffB: Matrix22 = Matrix22.zero,
      gyrOff: Vector2 = Vector2.zero,
      m: Vector2 = Vector2.zero
    ): MatrixD = MatrixD.horizontal(d, v, a, theta, omega, alpha, magOffB, gyrOff, m)

    val n: Int = 13
  }

  implicit class UF(val u: U) extends AnyVal {
    def FB: Vector2 = Vector2(u(0), u(1))

    def tau: Vector1 = Vector1(u(2))
  }

  object UF {
    def apply(FB: Vector2, tau: Vector1): U = U(VectorD.vertical(FB, tau))

    def columns(FB: MatrixD, tau: MatrixD): MatrixD = MatrixD.vertical(FB, tau)

    def oneRow(
      FB: Matrix12 = Matrix12.zero,
      tau: Vector1 = Vector1.zero
    ): MatrixD = MatrixD.horizontal(FB, tau)

    def twoRows(
      FB: Matrix22 = Matrix22.zero,
      tau: Vector2 = Vector2.zero
    ): MatrixD = MatrixD.horizontal(FB, tau)

    val n: Int = 3
  }

  implicit class ZF(val z: Z) extends AnyVal {
    def dis: Vector2 = Vector2(z(0), z(1))

    def vel: Vector2 = Vector2(z(2), z(3))

    def accB: Vector2 = Vector2(z(4), z(5))

    def magB: Vector2 = Vector2(z(6), z(7))

    def opt: Vector1 = Vector1(z(8))

    def gyr: Vector1 = Vector1(z(9))
  }

  object ZF {
    def apply(
      dis: Vector2,
      vel: Vector2,
      accB: Vector2,
      magB: Vector2,
      opt: Vector1,
      gyr: Vector1
    ): Z = Z(VectorD.vertical(dis, vel, accB, magB, opt, gyr))

    def columns(
      dis: MatrixD,
      vel: MatrixD,
      accB: MatrixD,
      magB: MatrixD,
      opt: MatrixD,
      gyr: MatrixD
    ): MatrixD = MatrixD.vertical(dis, vel, accB, magB, opt, gyr)

    def row(
      dis: Matrix22 = Matrix22.zero,
      vel: Matrix22 = Matrix22.zero,
      accB: Matrix22 = Matrix22.zero,
      magB: Matrix22 = Matrix22.zero,
      opt: Vector1 = Vector1.zero,
      gyr: Vector1 = Vector1.zero
    ): MatrixD = MatrixD.horizontal(dis, vel, accB, magB, opt, gyr)

    val n: Int = 10
  }

}
