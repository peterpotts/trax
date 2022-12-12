package com.peterpotts.trax.models.c

import com.peterpotts.trax.kalman._
import com.peterpotts.trax.math.MathDecorator._
import com.peterpotts.trax.math._

/**
  * @author Peter Potts
  */
case class PredictorC(mass: Double, dt: Double) extends Predictor {
  /**
    * The normal distributed process noise with covariance Q.
    */
  val w: U = UC(F = Vector2.constant(0.04))

  def predict(x: X, u: U): Prediction =
    Prediction(
      f = XC(
        d = x.d + x.v * dt + u.F * dt.squared / (2 * mass),
        v = x.v + u.F * dt / mass
      ),
      F = XC.columns(
        d = XC.row(d = Matrix22.identity, v = Matrix22.identity * dt),
        v = XC.row(v = Matrix22.identity)
      ),
      G = XC.columns(
        d = UC.row(F = Matrix22.identity * dt.squared / (2 * mass)),
        v = UC.row(F = Matrix22.identity * dt / mass)
      )
    )
}
