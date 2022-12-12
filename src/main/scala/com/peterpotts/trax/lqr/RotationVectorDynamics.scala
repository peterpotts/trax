package com.peterpotts.trax.lqr

import com.peterpotts.trax.math.MathDecorator._
import com.peterpotts.trax.math.MatrixDDecorator._
import com.peterpotts.trax.math._

/**
  * @author Peter Potts
  */
case class RotationVectorDynamics(inertiaB: DiagonalMatrix33, dt: Double) extends AngularDynamics3D {
  private val gamma = 10.0
  private val delta = 100.0

  private val B = MatrixD.vertical(
    MatrixD.horizontal(Matrix33.zero),
    MatrixD.horizontal(inertiaB.inverse * dt)
  )

  private val Q = MatrixD.vertical(
    MatrixD.horizontal(Matrix33.identity, Matrix33.zero),
    MatrixD.horizontal(Matrix33.zero, Matrix33.identity * gamma)
  )

  private val R = (inertiaB.squared.inverse.matrix33 * delta.squared).matrixD

  private var P = Q

  case class Gain(kr: Matrix33, ko: Matrix33)

  object Gain {
    val zero: Gain = gain(RotationVector.zero, Vector3.zero)
  }

  def gain(r: RotationVector, omegaB: Vector3): Gain = {
    val DARE: DiscreteAlgebraicRiccatiEquation = {
      val beta = RotationVector(omegaB * dt)
      val skew = (inertiaB.matrix33 * omegaB).skewSymmetric - omegaB.skewSymmetric * inertiaB.matrix33
      val m34 = (beta.unitQuaternion ∘ r.unitQuaternion).jacobianRotationVector

      DiscreteAlgebraicRiccatiEquation(
        nx = 6,
        nu = 3,
        A = MatrixD.vertical(
          MatrixD.horizontal(
            m34 * beta.unitQuaternion.quaternionMatrix.matrix44 * r.jacobianUnitQuaternion,
            m34 * r.unitQuaternion.conjugateQuaternionMatrix * beta.jacobianUnitQuaternion * dt
          ),
          MatrixD.horizontal(
            Matrix33.zero,
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

    val kr: Matrix33 = K.map(_.take(3)).matrix33
    val ko: Matrix33 = K.map(_.drop(3)).matrix33
    Gain(kr, ko)
  }
}
