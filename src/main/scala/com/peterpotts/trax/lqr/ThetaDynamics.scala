package com.peterpotts.trax.lqr

import com.peterpotts.trax.math.MathDecorator._
import com.peterpotts.trax.math._

/**
  * @author Peter Potts
  */
case class ThetaDynamics(inertia: Double, dt: Double) extends AngularDynamics2D {
  private val DARE: DiscreteAlgebraicRiccatiEquation = {
    val gamma = 100.0
    val delta = 500.0

    DiscreteAlgebraicRiccatiEquation(
      nx = 2,
      nu = 1,
      A = MatrixD(VectorD(1, dt), VectorD(0, 1)),
      B = MatrixD(VectorD(dt.squared / (2 * inertia)), VectorD(dt / inertia)),
      Q = MatrixD(VectorD(1, 0), VectorD(0, gamma)),
      R = MatrixD(VectorD(delta.squared / inertia.squared))
    )
  }

  val kt: Double = DARE.K.x.x
  val ko: Double = DARE.K.x.y
}
