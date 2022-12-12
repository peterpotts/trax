package com.peterpotts.trax.models.h

import com.peterpotts.trax.kalman._
import com.peterpotts.trax.math.MathDecorator._
import com.peterpotts.trax.math._

/**
  * @author Peter Potts
  */
case class PredictorH(mass: Double, inertiaB: DiagonalMatrix33, dt: Double) extends Predictor {
  val w: U = UH(
    FB = Vector3.constant(0.04),
    tauB = Vector3.constant(0.00001)
  )

  def predict(x: X, u: U): Prediction = {
    val beta = RotationVector(x.omegaB * dt + x.alphaB * dt.squared / 2)
    val q = beta.unitQuaternion ∘ x.q
    val RT = x.q.rotationMatrix.inverse.matrix33
    val twist = x.omegaB crossProduct (inertiaB * x.omegaB)
    val betaJacobian = x.q.conjugateQuaternionMatrix * beta.jacobianUnitQuaternion

    Prediction(
      f = XH(
        d = x.d + x.v * dt + x.a * dt.squared / 2,
        v = x.v + x.a * dt,
        a = RT * u.FB / mass,
        q = q,
        omegaB = x.omegaB + x.alphaB * dt,
        alphaB = inertiaB.inverse * (u.tauB - twist)
      ),
      F = XH.columns(
        d = XH.threeRows(d = Matrix33.identity, v = Matrix33.identity * dt, a = Matrix33.identity * dt.squared / 2),
        v = XH.threeRows(v = Matrix33.identity, a = Matrix33.identity * dt),
        a = XH.threeRows(q = x.q.jacobianRotationMatrix.kij * u.FB / mass),
        q = XH.fourRows(
          q = beta.quaternionMatrix.matrix44,
          omegaB = betaJacobian * dt,
          alphaB = betaJacobian * dt.squared / 2
        ),
        omegaB = XH.threeRows(omegaB = Matrix33.identity, alphaB = Matrix33.identity * dt),
        alphaB = XH.threeRows(
          omegaB = inertiaB.inverse * ((inertiaB * x.omegaB).skewSymmetric - x.omegaB.skewSymmetric * inertiaB)
        )
      ),
      G = XH.columns(
        d = UH.threeRows(),
        v = UH.threeRows(),
        a = UH.threeRows(FB = RT / mass),
        q = UH.fourRows(),
        omegaB = UH.threeRows(),
        alphaB = UH.threeRows(tauB = inertiaB.inverse.matrix33)
      )
    )
  }
}
