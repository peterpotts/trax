package com.peterpotts.trax.models.d

import com.peterpotts.trax.kalman._
import com.peterpotts.trax.lqr._
import com.peterpotts.trax.math.MatrixDDecorator._
import com.peterpotts.trax.math.VectorDDecorator._
import com.peterpotts.trax.math._
import com.peterpotts.trax.plan._

/**
  * @author Peter Potts
  */
case class ControllerD(mass: Double, inertia: Double, north: Vector2, dt: Double) extends Controller {
  private val planner = JourneyPlanner2D.example1(mass, inertia, dt)
  private val linearDynamics = LinearDynamics(mass, dt)
  private val angularDynamics = ThetaDynamics(inertia, dt)

  val initial: Target = toTarget(planner.initial)

  def target(time: Double, x: X, dt: Double): Target = toTarget(planner.plan(time, x, dt))

  private def toTarget(body: Body2D): Target = {
    val x =
      XD(
        d = body.d,
        v = body.v,
        a = body.a,
        theta = body.theta,
        omega = body.omega,
        alpha = body.alpha
      )

    val u =
      UD(
        FB = body.FB,
        tau = body.tau
      )

    Target(x, u)
  }

  def measurements(x: X, v: Z): Z = {
    val R = x.theta.rotationMatrix.matrix22

    ZD(
      dis = x.d,
      vel = x.v,
      accB = R * x.a,
      magB = R * north,
      gyr = x.alpha
    ) fuzz v
  }

  def regulate(x: X, target: Target): U = {
    val R = x.theta.rotationMatrix.matrix22

    val K =
      UD.columns(
        FB = XD.twoRows(
          d = R * linearDynamics.kd,
          v = R * linearDynamics.kv
        ),
        tau = XD.oneRow(
          theta = Vector1(angularDynamics.kt),
          omega = Vector1(angularDynamics.ko)
        )
      )

    val xe: X = x - target.x
    val ue: U = U(K ** xe.vectorD)
    U(target.u.vectorD - ue.vectorD)
  }
}
