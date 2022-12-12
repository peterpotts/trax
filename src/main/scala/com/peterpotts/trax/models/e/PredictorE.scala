package com.peterpotts.trax.models.e

import com.peterpotts.trax.kalman._
import com.peterpotts.trax.math.MathDecorator._
import com.peterpotts.trax.math._

/**
  * @author Peter Potts
  */
case class PredictorE(mass: Double, inertia: Double, dt: Double) extends Predictor {
  val w: U = UE(
    FB = Vector2.constant(0.04),
    tau = Vector1.constant(0.00001)
  )

  def predict(x: X, u: U): Prediction = {
    val RT = x.theta.rotationMatrix.inverse.matrix22
    val dRT = x.theta.jacobianRotationMatrix.inverse

    Prediction(
      f = XE(
        d = x.d + x.v * dt + x.a * dt.squared / 2,
        v = x.v + x.a * dt,
        a = RT * u.FB / mass,
        theta = x.theta + Angle(x.omega * dt + x.alpha * dt.squared / 2),
        omega = x.omega + x.alpha * dt,
        alpha = u.tau / inertia,
        magOffB = x.magOffB,
        gyrOff = x.gyrOff
      ),
      F = XE.columns(
        d = XE.twoRows(d = Matrix22.identity, v = Matrix22.identity * dt, a = Matrix22.identity * dt.squared / 2),
        v = XE.twoRows(v = Matrix22.identity, a = Matrix22.identity * dt),
        a = XE.twoRows(theta = dRT * u.FB / mass),
        theta = XE.oneRow(theta = Vector1.identity, omega = Vector1(dt), alpha = Vector1(dt.squared / 2)),
        omega = XE.oneRow(omega = Vector1.identity, alpha = Vector1(dt)),
        alpha = XE.oneRow(),
        magOffB = XE.twoRows(magOffB = Matrix22.identity),
        gyrOff = XE.oneRow(gyrOff = Vector1.identity)
      ),
      G = XE.columns(
        d = UE.twoRows(),
        v = UE.twoRows(),
        a = UE.twoRows(FB = RT / mass),
        theta = UE.oneRow(),
        omega = UE.oneRow(),
        alpha = UE.oneRow(tau = inertia.reciprocal.vector1),
        magOffB = UE.twoRows(),
        gyrOff = UE.oneRow()
      )
    )
  }
}
