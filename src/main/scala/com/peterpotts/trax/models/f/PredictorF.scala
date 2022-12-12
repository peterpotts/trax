package com.peterpotts.trax.models.f

import com.peterpotts.trax.kalman._
import com.peterpotts.trax.math.MathDecorator._
import com.peterpotts.trax.math._

/**
  * @author Peter Potts
  */
case class PredictorF(inertia: Double, dt: Double) extends Predictor {
  val w: U = UF(
    FB = Vector2.constant(0.04),
    tau = Vector1.constant(0.00001)
  )

  def predict(x: X, u: U): Prediction = {
    val RT = x.theta.rotationMatrix.inverse.matrix22
    val dRT = x.theta.jacobianRotationMatrix.inverse

    Prediction(
      f = XF(
        d = x.d + x.v * dt + x.a * dt.squared / 2,
        v = x.v + x.a * dt,
        a = RT * u.FB / x.m.value,
        theta = x.theta + Angle(x.omega * dt + x.alpha * dt.squared / 2),
        omega = x.omega + x.alpha * dt,
        alpha = u.tau / inertia,
        magOffB = x.magOffB,
        gyrOff = x.gyrOff,
        m = x.m
      ),
      F = XF.columns(
        d = XF.twoRows(d = Matrix22.identity, v = Matrix22.identity * dt, a = Matrix22.identity * dt.squared / 2),
        v = XF.twoRows(v = Matrix22.identity, a = Matrix22.identity * dt),
        a = XF.twoRows(theta = dRT * u.FB / x.m.value, m = RT * u.FB / -x.m.value.squared),
        theta = XF.oneRow(theta = Vector1.identity, omega = Vector1(dt), alpha = Vector1(dt.squared / 2)),
        omega = XF.oneRow(omega = Vector1.identity, alpha = Vector1(dt)),
        alpha = XF.oneRow(),
        magOffB = XF.twoRows(magOffB = Matrix22.identity),
        gyrOff = XF.oneRow(gyrOff = Vector1.identity),
        m = XF.oneRow(gyrOff = Vector1.identity)
      ),
      G = XF.columns(
        d = UF.twoRows(),
        v = UF.twoRows(),
        a = UF.twoRows(FB = RT / x.m),
        theta = UF.oneRow(),
        omega = UF.oneRow(),
        alpha = UF.oneRow(tau = inertia.reciprocal.vector1),
        magOffB = UF.twoRows(),
        gyrOff = UF.oneRow(),
        m = UF.oneRow()
      )
    )
  }
}
