package com.peterpotts.trax.models.c

import com.peterpotts.trax.kalman._
import com.peterpotts.trax.lqr.LinearDynamics
import com.peterpotts.trax.math.MatrixDDecorator._
import com.peterpotts.trax.math.VectorDDecorator._
import com.peterpotts.trax.math._
import com.peterpotts.trax.plan._

/**
  * @author Peter Potts
  */
case class ControllerC2(mass: Double, inertia: Double, dt: Double) extends Controller {
  private val planner = {
    val initial = Body2D(mass = mass, inertia = inertia)

    val legs = List(
      Leg2D(400, (_: Body2D, _: Double) => new Body2D(
        mass = mass,
        d = Vector2.X * 80,
        v = Vector2.zero,
        FB = Vector2.zero,
        inertia = inertia,
        theta = Angle.zero,
        omega = Vector1.zero,
        tau = Vector1.zero
      ))
    )

    JourneyPlanner2D(initial, legs, dt)
  }

  private def toTarget(body: Body2D): Target = {
    val x = XC(d = body.d, v = body.v)
    val u = UC(F = body.F)
    // TODO println(s"d = ${body.d} / v = ${body.v} / F = ${body.F}")
    Target(x, u)
  }

  /////////////////
  // INITIALIZER //
  /////////////////

  val initial: Target = toTarget(planner.initial)

  /////////////
  // PLANNER //
  /////////////

  def target(time: Double, x: X, dt: Double): Target = toTarget(planner.plan(time, x, dt))

  ////////////
  // SENSOR //
  ////////////

  def measurements(x: X, v: Z): Z = ZC(dis = x.d) fuzz v

  ///////////////
  // REGULATOR //
  ///////////////

  private val linearDynamics = LinearDynamics(mass, dt)

  private val K =
    UC.columns(
      F = XC.row(
        d = Matrix22.identity * linearDynamics.kd,
        v = Matrix22.identity * linearDynamics.kv
      )
    )

  private def limit(u: U): U = u // U(u.vectorD.map(_ max -10 min 10))

  def regulate(x: X, target: Target): U = {
    val xe: X = x - target.x
    val ue: U = U(K ** xe.vectorD)
    limit(U(target.u.vectorD - ue.vectorD))
  }
}
