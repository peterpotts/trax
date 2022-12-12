package com.peterpotts.trax.models

import com.peterpotts.trax.kalman._
import com.peterpotts.trax.math._

/**
  * @author Peter Potts
  */
package object h {

  implicit class XH(val x: X) extends AnyVal {
    def d: Vector3 = Vector3(x(0), x(1), x(2))

    def v: Vector3 = Vector3(x(3), x(4), x(5))

    def a: Vector3 = Vector3(x(6), x(7), x(8))

    def q: UnitQuaternion = Quaternion(Vector4(x(9), x(10), x(11), x(12))).versor

    def omegaB: Vector3 = Vector3(x(13), x(14), x(15))

    def alphaB: Vector3 = Vector3(x(16), x(17), x(18))

    def -(that: X): X = XH(
      d = d - that.d,
      v = v - that.v,
      a = a - that.a,
      q = q ∘ that.q.inverse,
      omegaB = omegaB - that.omegaB,
      alphaB = alphaB - that.alphaB
    )
  }

  object XH {
    def apply(
      d: Vector3,
      v: Vector3,
      a: Vector3,
      q: UnitQuaternion,
      omegaB: Vector3,
      alphaB: Vector3
    ): X = X(VectorD.vertical(d, v, a, q.quaternion.vector4, omegaB, alphaB))

    def columns(
      d: MatrixD,
      v: MatrixD,
      a: MatrixD,
      q: MatrixD,
      omegaB: MatrixD,
      alphaB: MatrixD
    ): MatrixD = MatrixD.vertical(d, v, a, q, omegaB, alphaB)

    def threeRows(
      d: Matrix33 = Matrix33.zero,
      v: Matrix33 = Matrix33.zero,
      a: Matrix33 = Matrix33.zero,
      q: Matrix34 = Matrix34.zero,
      omegaB: Matrix33 = Matrix33.zero,
      alphaB: Matrix33 = Matrix33.zero
    ): MatrixD = MatrixD.horizontal(d, v, a, q, omegaB, alphaB)

    def fourRows(
      d: Matrix43 = Matrix43.zero,
      v: Matrix43 = Matrix43.zero,
      a: Matrix43 = Matrix43.zero,
      q: Matrix44 = Matrix44.zero,
      omegaB: Matrix43 = Matrix43.zero,
      alphaB: Matrix43 = Matrix43.zero
    ): MatrixD = MatrixD.horizontal(d, v, a, q, omegaB, alphaB)

    val n: Int = 19
  }

  implicit class UH(val u: U) extends AnyVal {
    def FB: Vector3 = Vector3(u(0), u(1), u(2))

    def tauB: Vector3 = Vector3(u(3), u(4), u(5))
  }

  object UH {
    def apply(FB: Vector3, tauB: Vector3): U = U(VectorD.vertical(FB, tauB))

    def columns(FB: MatrixD, tauB: MatrixD): MatrixD = MatrixD.vertical(FB, tauB)

    def threeRows(FB: Matrix33 = Matrix33.zero, tauB: Matrix33 = Matrix33.zero): MatrixD = MatrixD.horizontal(FB, tauB)

    def fourRows(FB: Matrix43 = Matrix43.zero, tauB: Matrix43 = Matrix43.zero): MatrixD = MatrixD.horizontal(FB, tauB)

    val n: Int = 6
  }

  implicit class ZH(val z: Z) extends AnyVal {
    def dis: Vector3 = Vector3(z(0), z(1), z(2))

    def vel: Vector3 = Vector3(z(3), z(4), z(5))

    def accB: Vector3 = Vector3(z(6), z(7), z(8))

    def magB: Vector3 = Vector3(z(9), z(10), z(11))

    def optB: Vector3 = Vector3(z(12), z(13), z(14))

    def gyrB: Vector3 = Vector3(z(15), z(16), z(17))
  }

  object ZH {
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
