package com.peterpotts.trax.plan

import com.peterpotts.trax.math._

/**
  * @author Peter Potts
  */
case class Body2D(
  mass: Double,
  d: Vector2,
  v: Vector2,
  FB: Vector2,
  inertia: Double,
  theta: Angle,
  omega: Vector1,
  tau: Vector1
) {
  ////////////
  // Linear //
  ////////////

  lazy val a: Vector2 = F / mass
  lazy val p: Vector2 = v * mass
  lazy val F: Vector2 = RT * FB

  /////////////
  // Angular //
  /////////////

  def R: Matrix22 = theta.rotationMatrix.matrix22

  def RT: Matrix22 = theta.rotationMatrix.inverse.matrix22

  lazy val alpha: Vector1 = tau / inertia
  lazy val L: Vector1 = omega * inertia
}

object Body2D {
  def apply(mass: Double, inertia: Double): Body2D =
    Body2D(
      mass = mass,
      d = Vector2.zero,
      v = Vector2.zero,
      FB = Vector2.zero,
      inertia = inertia,
      theta = Angle.zero,
      omega = Vector1.zero,
      tau = Vector1.zero
    )
}
