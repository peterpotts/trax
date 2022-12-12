package com.peterpotts.trax.plan

import com.peterpotts.trax.kalman.X
import com.peterpotts.trax.math._

/**
  * @author Peter Potts
  */
case class CircularPlanner2D(mass: Double, inertia: Double) extends Planner2D {
  val radius: Double = 10.0
  val period: Double = 100.0
  val angularVelocity: Double = TwoPi / period
  val velocity: Double = radius * angularVelocity
  val acceleration: Double = velocity * angularVelocity
  val force: Double = mass * acceleration

  private val d = Vector2.X * radius
  private val vB = Vector2.X * velocity
  private val FB = Vector2.Y * force
  private val omega = Vector1(angularVelocity)

  val initial: Body2D = project(0)

  def plan(time: Double, x: X, dt: Double): Body2D = project(time + dt)

  private def project(time: Double): Body2D = {
    val bearing = Angle.normalize(angularVelocity * time)
    val theta = bearing + Angle.right

    Body2D(
      mass = mass,
      d = bearing.rotationMatrix.inverse.matrix22 * d,
      v = theta.rotationMatrix.inverse.matrix22 * vB,
      FB = FB,
      inertia = inertia,
      theta = theta,
      omega = omega,
      tau = Vector1.zero
    )
  }
}
