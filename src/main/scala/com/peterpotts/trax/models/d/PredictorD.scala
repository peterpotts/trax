package com.peterpotts.trax.models.d

import com.peterpotts.trax.kalman._
import com.peterpotts.trax.math.MathDecorator._
import com.peterpotts.trax.math._

/**
  * @author Peter Potts
  */
case class PredictorD(mass: Double, inertia: Double, dt: Double) extends Predictor {
  val w: U = UD(
    FB = Vector2.constant(0.04),
    tau = Vector1.constant(0.00001)
  )

  def predict(x: X, u: U): Prediction = {
    val RT = x.theta.rotationMatrix.inverse.matrix22
    val dRT = x.theta.jacobianRotationMatrix.inverse

    Prediction(
      f = XD(
        d = x.d + x.v * dt + x.a * dt.squared / 2,
        v = x.v + x.a * dt,
        a = RT * u.FB / mass,
        theta = x.theta + Angle(x.omega * dt + x.alpha * dt.squared / 2),
        omega = x.omega + x.alpha * dt,
        alpha = u.tau / inertia
      ),
      F = XD.columns(
        d = XD.twoRows(d = Matrix22.identity, v = Matrix22.identity * dt, a = Matrix22.identity * dt.squared / 2),
        v = XD.twoRows(v = Matrix22.identity, a = Matrix22.identity * dt),
        a = XD.twoRows(theta = dRT * u.FB / mass),
        theta = XD.oneRow(theta = Vector1.identity, omega = Vector1(dt), alpha = Vector1(dt.squared / 2)),
        omega = XD.oneRow(omega = Vector1.identity, alpha = Vector1(dt)),
        alpha = XD.oneRow()
      ),
      G = XD.columns(
        d = UD.twoRows(),
        v = UD.twoRows(),
        a = UD.twoRows(FB = RT / mass),
        theta = UD.oneRow(),
        omega = UD.oneRow(),
        alpha = UD.oneRow(tau = inertia.reciprocal.vector1)
      )
    )
  }
}
