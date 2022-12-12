package com.peterpotts.trax.models.j

import com.peterpotts.trax.kalman._
import com.peterpotts.trax.math.MathDecorator._
import com.peterpotts.trax.math._

/**
  * @author Peter Potts
  */
case class PredictorJ(mass: Double, inertiaB: DiagonalMatrix33, dt: Double) extends Predictor {
  val w: U = UJ(
    FB = Vector3.constant(0.04),
    tauB = Vector3.constant(0.00001)
  )

  def predict(x: X, u: U): Prediction = {
    val beta = RotationVector(x.omegaB * dt + x.alphaB * dt.squared / 2)
    val q = beta.unitQuaternion ∘ x.r.unitQuaternion
    val RT = x.r.rotationMatrix.inverse.matrix33
    val twist = x.omegaB crossProduct (inertiaB * x.omegaB)
    val rJacobian = q.jacobianRotationVector * beta.quaternionMatrix * x.r.jacobianUnitQuaternion
    val betaJacobian = q.jacobianRotationVector * x.r.conjugateQuaternionMatrix * beta.jacobianUnitQuaternion

    Prediction(
      f = XJ(
        d = x.d + x.v * dt + x.a * dt.squared / 2,
        v = x.v + x.a * dt,
        a = RT * u.FB / mass,
        r = q.rotationVector,
        omegaB = x.omegaB + x.alphaB * dt,
        alphaB = inertiaB.inverse * (u.tauB - twist),
        g = x.g,
        N = x.N
      ),
      F = XJ.columns(
        d = XJ.row(d = Matrix33.identity, v = Matrix33.identity * dt, a = Matrix33.identity * dt.squared / 2),
        v = XJ.row(v = Matrix33.identity, a = Matrix33.identity * dt),
        a = XJ.row(r = x.r.jacobianRotationMatrix.kij * u.FB / mass),
        r = XJ.row(
          r = rJacobian,
          omegaB = betaJacobian * dt,
          alphaB = betaJacobian * dt.squared / 2
        ),
        omegaB = XJ.row(omegaB = Matrix33.identity, alphaB = Matrix33.identity * dt),
        alphaB = XJ.row(
          omegaB = inertiaB.inverse * ((inertiaB * x.omegaB).skewSymmetric - x.omegaB.skewSymmetric * inertiaB)
        ),
        g = XJ.row(g = Matrix33.identity),
        N = XJ.row(N = Matrix33.identity)
      ),
      G = XJ.columns(
        d = UJ.row(),
        v = UJ.row(),
        a = UJ.row(FB = RT / mass),
        r = UJ.row(),
        omegaB = UJ.row(),
        alphaB = UJ.row(tauB = inertiaB.inverse.matrix33),
        g = UJ.row(),
        N = UJ.row()
      )
    )
  }
}
