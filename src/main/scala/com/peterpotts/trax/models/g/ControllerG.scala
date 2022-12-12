package com.peterpotts.trax.models.g

import com.peterpotts.trax.kalman._
import com.peterpotts.trax.lqr._
import com.peterpotts.trax.math.VectorDDecorator._
import com.peterpotts.trax.math._
import com.peterpotts.trax.plan._

/**
  * @author Peter Potts
  */
case class ControllerG(mass: Double, inertia: Double, north: Vector2, dt: Double) extends Controller {
  private val planner = JourneyPlanner2D.example1(mass, inertia, dt)
  private val linearDynamics = LinearDynamics(mass, dt)
  private val angularDynamics = ThetaDynamics(inertia, dt)

  val initial: Target = toTarget(planner.initial)

  def target(time: Double, x: X, dt: Double): Target = toTarget(planner.plan(time, x, dt))

  private def toTarget(body: Body2D): Target = {
    val x =
      XG(
        d = body.d,
        v = body.v,
        a = body.a,
        theta = body.theta,
        omega = body.omega,
        alpha = body.alpha
      )

    val u =
      UG(
        FB = body.FB,
        tau = body.tau
      )

    Target(x, u)
  }

  def measurements(x: X, v: Z): Z = {
    val R = x.theta.rotationMatrix.matrix22

    ZG(
      dis = x.d,
      vel = x.v,
      accB = R * x.a,
      magB = R * north,
      opt = x.omega,
      gyr = x.alpha
    ) fuzz v
  }

  def regulate(x: X, target: Target): U = {
    val R = x.theta.rotationMatrix.matrix22
    val xe = x - target.x

    val ue =
      UG(
        FB = R * (xe.d * linearDynamics.kd + xe.v * linearDynamics.kv),
        tau = xe.theta.vector1 * angularDynamics.kt + xe.omega * angularDynamics.ko
      )

    U(target.u.vectorD - ue.vectorD)
  }
}
