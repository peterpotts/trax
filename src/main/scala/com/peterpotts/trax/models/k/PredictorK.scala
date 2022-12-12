package com.peterpotts.trax.models.k

import com.peterpotts.trax.kalman._
import com.peterpotts.trax.math.MathDecorator._
import com.peterpotts.trax.math._

case class PredictorK(mass: Double, inertiaB: DiagonalMatrix33, dt: Double) extends Predictor {
  val w: U = UK(
    FB = Vector3.constant(0.04),
    tauB = Vector3.constant(0.00001)
  )

  def predict(x: X, u: U): Prediction = {
    val beta = x.omegaB * dt + x.alphaB * dt.squared / 2
    val e = EulerAngles.normalize(x.e.vector3 + x.e.inverseConjugateRatesMatrix * beta)
    val RT = x.e.rotationMatrix.inverse.matrix33
    val twist = x.omegaB crossProduct (inertiaB * x.omegaB)

    Prediction(
      f = XK(
        d = x.d + x.v * dt + x.a * dt.squared / 2,
        v = x.v + x.a * dt,
        a = RT * u.FB / mass,
        e = e,
        omegaB = x.omegaB + x.alphaB * dt,
        alphaB = inertiaB.inverse * (u.tauB - twist),
        g = x.g,
        N = x.N
      ),
      F = XK.columns(
        d = XK.row(d = Matrix33.identity, v = Matrix33.identity * dt, a = Matrix33.identity * dt.squared / 2),
        v = XK.row(v = Matrix33.identity, a = Matrix33.identity * dt),
        a = XK.row(e = x.e.jacobianRotationMatrix.kij * u.FB / mass),
        e = XK.row(
          e = Matrix33.identity + e.jacobianInverseConjugateRatesMatrix.jki * beta,
          omegaB = e.inverseConjugateRatesMatrix * dt,
          alphaB = e.inverseConjugateRatesMatrix * dt.squared / 2
        ),
        omegaB = XK.row(omegaB = Matrix33.identity, alphaB = Matrix33.identity * dt),
        alphaB = XK.row(
          omegaB = inertiaB.inverse * ((inertiaB * x.omegaB).skewSymmetric - x.omegaB.skewSymmetric * inertiaB)
        ),
        g = XK.row(g = Matrix33.identity),
        N = XK.row(N = Matrix33.identity)
      ),
      G = XK.columns(
        d = UK.row(),
        v = UK.row(),
        a = UK.row(FB = RT / mass),
        e = UK.row(),
        omegaB = UK.row(),
        alphaB = UK.row(tauB = inertiaB.inverse.matrix33),
        g = UK.row(),
        N = UK.row()
      )
    )
  }
}
