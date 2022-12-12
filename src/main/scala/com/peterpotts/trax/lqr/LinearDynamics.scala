package com.peterpotts.trax.lqr

import com.peterpotts.trax.math.MathDecorator._
import com.peterpotts.trax.math._

/**
  * @author Peter Potts
  */
case class LinearDynamics(mass: Double, dt: Double) {
  private val DARE: DiscreteAlgebraicRiccatiEquation = {
    val alpha = 1.0
    val beta = 10.0 // Higher values of beta give slower convergence

    DiscreteAlgebraicRiccatiEquation(
      nx = 2,
      nu = 1,
      A = MatrixD(VectorD(1, dt), VectorD(0, 1)),
      B = MatrixD(VectorD(dt.squared / (2 * mass)), VectorD(dt / mass)),
      Q = MatrixD(VectorD(1, 0), VectorD(0, alpha)),
      R = MatrixD(VectorD(beta.squared / mass.squared))
    )
  }

  val kd: Double = DARE.K.x.x
  val kv: Double = DARE.K.x.y
}
