package com.peterpotts.trax.lqr

import com.peterpotts.trax.math.MathDecorator._
import com.peterpotts.trax.math.MatrixDDecorator._
import com.peterpotts.trax.math._

/**
  * @author Peter Potts
  */
case class QuaternionDynamics(inertiaB: DiagonalMatrix33, dt: Double) extends AngularDynamics3D {
  private val gamma = 10.0
  private val delta = 100.0

  private val B = MatrixD.vertical(
    MatrixD.horizontal(Matrix43.zero),
    MatrixD.horizontal(inertiaB.inverse * dt)
  )

  private val Q = MatrixD.vertical(
    MatrixD.horizontal(Matrix44.identity, Matrix43.zero),
    MatrixD.horizontal(Matrix34.zero, Matrix33.identity * gamma)
  )

  private val R = (inertiaB.squared.inverse.matrix33 * delta.squared).matrixD

  private var P = Q

  case class Gain(kq: Matrix34, ko: Matrix33)

  object Gain {
    val zero: Gain = gain(UnitQuaternion.W, Vector3.zero)
  }

  def gain(q: UnitQuaternion, omegaB: Vector3): Gain = {
    val DARE: DiscreteAlgebraicRiccatiEquation = {
      val beta = RotationVector(omegaB * dt)
      val skew = (inertiaB.matrix33 * omegaB).skewSymmetric - omegaB.skewSymmetric * inertiaB.matrix33

      DiscreteAlgebraicRiccatiEquation(
        nx = 7,
        nu = 3,
        A = MatrixD.vertical(
          MatrixD.horizontal(
            beta.unitQuaternion.quaternionMatrix.matrix44,
            q.conjugateQuaternionMatrix.matrix44 * beta.jacobianUnitQuaternion * dt
          ),
          MatrixD.horizontal(
            Matrix34.zero,
            Matrix33.identity + inertiaB.inverse * skew * dt
          )
        ),
        B = B,
        Q = Q,
        R = R
      )
    }

    val K = {
      P = DARE.iterate(P)
      DARE.gain(P)
    }

    println(for (index <- P.indices) yield P(index)(index))
    println("")
    for (index <- P.indices) yield println(P(index))

    val kq: Matrix34 = K.map(_.take(4)).matrix34
    val ko: Matrix33 = K.map(_.drop(4)).matrix33
    Gain(kq, ko)
  }

}
