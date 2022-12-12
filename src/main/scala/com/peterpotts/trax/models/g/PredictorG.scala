package com.peterpotts.trax.models.g

import com.peterpotts.trax.kalman._
import com.peterpotts.trax.math.MathDecorator._
import com.peterpotts.trax.math._

/**
  * @author Peter Potts
  */
case class PredictorG(mass: Double, inertia: Double, dt: Double) extends Predictor {
  val w: U = UG(
    FB = Vector2.constant(0.04),
    tau = Vector1.constant(0.00001)
  )

  def predict(x: X, u: U): Prediction = {
    val RT = x.theta.rotationMatrix.inverse.matrix22
    val dRT = x.theta.jacobianRotationMatrix.transpose

    Prediction(
      f = XG(
        d = x.d + x.v * dt + x.a * dt.squared / 2,
        v = x.v + x.a * dt,
        a = RT * u.FB / mass,
        theta = x.theta + Angle(x.omega * dt + x.alpha * dt.squared / 2),
        omega = x.omega + x.alpha * dt,
        alpha = u.tau / inertia
      ),
      F = XG.columns(
        d = XG.twoRows(d = Matrix22.identity, v = Matrix22.identity * dt, a = Matrix22.identity * dt.squared / 2),
        v = XG.twoRows(v = Matrix22.identity, a = Matrix22.identity * dt),
        a = XG.twoRows(theta = dRT * u.FB / mass),
        theta = XG.oneRow(theta = Vector1.identity, omega = Vector1(dt), alpha = Vector1(dt.squared / 2)),
        omega = XG.oneRow(omega = Vector1.identity, alpha = Vector1(dt)),
        alpha = XG.oneRow()
      ),
      G = XG.columns(
        d = UG.twoRows(),
        v = UG.twoRows(),
        a = UG.twoRows(FB = RT / mass),
        theta = UG.oneRow(),
        omega = UG.oneRow(),
        alpha = UG.oneRow(tau = inertia.reciprocal.vector1)
      )
    )
  }
}
