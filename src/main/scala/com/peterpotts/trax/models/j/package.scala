package com.peterpotts.trax.models

import com.peterpotts.trax.kalman._
import com.peterpotts.trax.math._

/**
  * @author Peter Potts
  */
package object j {

  implicit class XJ(val x: X) extends AnyVal {
    def d: Vector3 = Vector3(x(0), x(1), x(2))

    def v: Vector3 = Vector3(x(3), x(4), x(5))

    def a: Vector3 = Vector3(x(6), x(7), x(8))

    def r: RotationVector = RotationVector.normalize(Vector3(x(9), x(10), x(11)))

    def omegaB: Vector3 = Vector3(x(12), x(13), x(14))

    def alphaB: Vector3 = Vector3(x(15), x(16), x(17))

    def g: Vector3 = Vector3(x(18), x(19), x(20))

    def N: Vector3 = Vector3(x(21), x(22), x(23))

    def -(that: X): X = XJ(
      d = d - that.d,
      v = v - that.v,
      a = a - that.a,
      r = r * that.r.inverse,
      omegaB = omegaB - that.omegaB,
      alphaB = alphaB - that.alphaB,
      g = g - that.g,
      N = N - that.N
    )
  }

  object XJ {
    def apply(
      d: Vector3,
      v: Vector3,
      a: Vector3,
      r: RotationVector,
      omegaB: Vector3,
      alphaB: Vector3,
      g: Vector3,
      N: Vector3
    ): X = X(VectorD.vertical(d, v, a, r.vector3, omegaB, alphaB, g, N))

    def columns(
      d: MatrixD,
      v: MatrixD,
      a: MatrixD,
      r: MatrixD,
      omegaB: MatrixD,
      alphaB: MatrixD,
      g: MatrixD,
      N: MatrixD
    ): MatrixD = MatrixD.vertical(d, v, a, r, omegaB, alphaB, g, N)

    def row(
      d: Matrix33 = Matrix33.zero,
      v: Matrix33 = Matrix33.zero,
      a: Matrix33 = Matrix33.zero,
      r: Matrix33 = Matrix33.zero,
      omegaB: Matrix33 = Matrix33.zero,
      alphaB: Matrix33 = Matrix33.zero,
      g: Matrix33 = Matrix33.zero,
      N: Matrix33 = Matrix33.zero
    ): MatrixD = MatrixD.horizontal(d, v, a, r, omegaB, alphaB, g, N)

    val n: Int = 24
  }

  implicit class UJ(val u: U) extends AnyVal {
    def FB: Vector3 = Vector3(u(0), u(1), u(2))

    def tauB: Vector3 = Vector3(u(3), u(4), u(5))
  }

  object UJ {
    def apply(FB: Vector3, tauB: Vector3): U = U(VectorD.vertical(FB, tauB))

    def columns(FB: MatrixD, tauB: MatrixD): MatrixD = MatrixD.vertical(FB, tauB)

    def row(FB: Matrix33 = Matrix33.zero, tauB: Matrix33 = Matrix33.zero): MatrixD =
      MatrixD.horizontal(FB, tauB)

    val n: Int = 6
  }

  implicit class ZJ(val z: Z) extends AnyVal {
    def dis: Vector3 = Vector3(z(0), z(1), z(2))

    def vel: Vector3 = Vector3(z(3), z(4), z(5))

    def accB: Vector3 = Vector3(z(6), z(7), z(8))

    def magB: Vector3 = Vector3(z(9), z(10), z(11))

    def optB: Vector3 = Vector3(z(12), z(13), z(14))

    def gyrB: Vector3 = Vector3(z(15), z(16), z(17))
  }

  object ZJ {
    def apply(
      dis: Vector3,
      vel: Vector3,
      accB: Vector3,
      magB: Vector3,
      optB: Vector3,
      gyrB: Vector3
    ): Z = Z(VectorD.vertical(dis, vel, accB, magB, optB, gyrB))

    def columns(
      dis: MatrixD,
      vel: MatrixD,
      accB: MatrixD,
      magB: MatrixD,
      optB: MatrixD,
      gyrB: MatrixD
    ): MatrixD = MatrixD.vertical(dis, vel, accB, magB, optB, gyrB)

    def row(
      dis: Matrix33 = Matrix33.zero,
      vel: Matrix33 = Matrix33.zero,
      accB: Matrix33 = Matrix33.zero,
      magB: Matrix33 = Matrix33.zero,
      optB: Matrix33 = Matrix33.zero,
      gyrB: Matrix33 = Matrix33.zero
    ): MatrixD = MatrixD.horizontal(dis, vel, accB, magB, optB, gyrB)

    val n: Int = 18
  }

}
