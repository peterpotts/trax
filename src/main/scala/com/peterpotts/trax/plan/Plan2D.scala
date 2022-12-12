package com.peterpotts.trax.plan

import com.peterpotts.trax.math.MathDecorator._
import com.peterpotts.trax.math._

/**
  * @author Peter Potts
  */
trait Plan2D {
  def apply(body: Body2D, dt: Double): Body2D
}

object Cruise2D extends Plan2D {
  def apply(body: Body2D, dt: Double): Body2D =
    body.copy(
      d = body.d + body.v * dt,
      FB = Vector2.zero,
      theta = body.theta + Angle.normalize(body.omega * dt),
      tau = Vector1.zero
    )
}

case class Force2D(FB: Vector2) extends Plan2D {
  def apply(body: Body2D, dt: Double): Body2D =
    if (body.omega === Vector1.zero) {
      val F = body.theta.rotationMatrix.inverse.matrix22 * FB
      val a = F / body.mass
      val v = body.v + a * dt
      val d = body.d + body.v * dt + a * dt.squared / 2

      body.copy(
        d = d,
        v = v,
        FB = FB,
        tau = Vector1.zero
      )
    } else {
      val delta = Angle.normalize(body.omega * dt)
      val theta = body.theta + delta
      val K = FB / (body.mass * body.omega.value)
      val S = body.theta integrateRotationMatrix theta
      val v = body.v + S.transpose * K
      val L = body.v + body.theta.jacobianRotationMatrix.transpose * K
      val J = body.theta.rotationMatrix.matrix22 - theta.rotationMatrix.matrix22
      val d = body.d + L * dt + J.transpose * K / body.omega.value

      body.copy(
        d = d,
        v = v,
        FB = FB,
        theta = theta,
        tau = Vector1.zero
      )
    }
}

case class Torque2D(tau: Vector1) extends Plan2D {
  def apply(body: Body2D, dt: Double): Body2D = {
    body.copy(
      d = body.d + body.v * dt,
      FB = Vector2.zero,
      theta = body.theta + Angle.normalize(body.omega * dt + tau * dt.squared / (2 * body.inertia)),
      omega = body.omega + tau * dt / body.inertia,
      tau = tau
    )
  }
}
