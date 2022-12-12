package com.peterpotts.trax.lqr

import com.peterpotts.trax.math.MathDecorator._
import com.peterpotts.trax.math.MatrixDDecorator._
import com.peterpotts.trax.math._

/**
  * @author Peter Potts
  */
case class EulerAnglesDynamics(inertiaB: DiagonalMatrix33, dt: Double) extends AngularDynamics3D {
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

  case class Gain(ke: Matrix33, ko: Matrix33)

  object Gain {
    val zero: Gain = gain(EulerAngles.zero, Vector3.zero)
  }

  def gain(e: EulerAngles, omegaB: Vector3): Gain = {
    val DARE: DiscreteAlgebraicRiccatiEquation = {
      val skew = (inertiaB.matrix33 * omegaB).skewSymmetric - omegaB.skewSymmetric * inertiaB.matrix33

      DiscreteAlgebraicRiccatiEquation(
        nx = 6,
        nu = 3,
        A = MatrixD.vertical(
          MatrixD.horizontal(
            Matrix33.identity + e.jacobianInverseConjugateRatesMatrix.ikj * omegaB * dt,
            e.inverseConjugateRatesMatrix * dt
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

    val ke: Matrix33 = K.map(_.take(3)).matrix33
    val ko: Matrix33 = K.map(_.drop(3)).matrix33
    Gain(ke, ko)
  }
}
