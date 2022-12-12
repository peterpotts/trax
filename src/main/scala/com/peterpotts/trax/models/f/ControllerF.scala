package com.peterpotts.trax.models.f

import com.peterpotts.trax.kalman._
import com.peterpotts.trax.lqr._
import com.peterpotts.trax.math.MatrixDDecorator._
import com.peterpotts.trax.math.VectorDDecorator._
import com.peterpotts.trax.math._
import com.peterpotts.trax.plan._

/**
  * @author Peter Potts
  */
case class ControllerF(
  mass: Double,
  inertia: Double,
  north: Vector2,
  magOffB: Vector2,
  gyrOff: Vector1,
  dt: Double
) extends Controller {
  private val planner = JourneyPlanner2D.example1(mass, inertia, dt)
  private val linearDynamics = LinearDynamics(mass, dt)
  private val angularDynamics = ThetaDynamics(inertia, dt)

  val initial: Target = toTarget(planner.initial)

  def target(time: Double, x: X, dt: Double): Target = toTarget(planner.plan(time, x, dt))

  private def toTarget(body: Body2D): Target = {
    val x =
      XF(
        d = body.d,
        v = body.v,
        a = body.a,
        theta = body.theta,
        omega = body.omega,
        alpha = body.alpha,
        magOffB = magOffB,
        gyrOff = gyrOff,
        m = Vector1(mass)
      )

    val u =
      UF(
        FB = body.FB,
        tau = body.tau
      )

    Target(x, u)
  }

  def measurements(x: X, v: Z): Z = {
    val R = x.theta.rotationMatrix.matrix22

    ZF(
      dis = x.d,
      vel = x.v,
      accB = R * x.a,
      magB = R * north + magOffB,
      opt = x.omega,
      gyr = x.alpha + gyrOff
    ) fuzz v
  }

  def regulate(x: X, target: Target): U = {
    val R = x.theta.rotationMatrix.matrix22

    val K =
      UF.columns(
        FB = XF.twoRows(
          d = R * linearDynamics.kd,
          v = R * linearDynamics.kv
        ),
        tau = XF.oneRow(
          theta = Vector1(angularDynamics.kt),
          omega = Vector1(angularDynamics.ko)
        )
      )

    val xe: X = x - target.x
    val ue: U = U(K ** xe.vectorD)
    U(target.u.vectorD - ue.vectorD)
  }
}
