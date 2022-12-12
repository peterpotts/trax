package com.peterpotts.trax.plan

import com.peterpotts.trax.kalman.X
import com.peterpotts.trax.math._

/**
  * @author Peter Potts
  */
case class CircularPlanner3D(mass: Double, inertiaB: DiagonalMatrix33) extends Planner3D {
  val radius: Double = 10.0
  val period: Double = 100.0
  val angularVelocity: Double = TwoPi / period
  val velocity: Double = radius * angularVelocity
  val acceleration: Double = velocity * angularVelocity
  val force: Double = mass * acceleration

  private val d = Vector3.X * radius
  private val vB = Vector3.X * velocity
  private val FB = Vector3.Y * force
  private val omegaB = Vector3.Z * angularVelocity

  val initial: Body3D = project(0)

  def plan(time: Double, x: X, dt: Double): Body3D = project(time + dt)

  private def project(time: Double): Body3D = {
    val bearing = Angle.normalize(angularVelocity * time)
    val theta = bearing + Angle.right

    Body3D(
      mass = mass,
      d = bearing.z.rotationMatrix.inverse.matrix33 * d,
      v = theta.z.rotationMatrix.inverse.matrix33 * vB,
      FB = FB,
      inertiaB = inertiaB,
      q = theta.z.unitQuaternion,
      omegaB = omegaB,
      tauB = Vector3.zero
    )
  }
}
