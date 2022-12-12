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
case class ControllerC(mass: Double, inertia: Double, dt: Double) extends Controller {
  private val planner = JourneyPlanner2D.example1(mass, inertia, dt)
  // private val planner = JourneyPlanner2D.example3(mass, inertia, dt)
  private val linearDynamics = LinearDynamics(mass, dt)

  val initial: Target = toTarget(planner.initial)

  def target(time: Double, x: X, dt: Double): Target = toTarget(planner.plan(time, x, dt))

  private def toTarget(body: Body2D): Target = {
    val x = XC(d = body.d, v = body.v)
    val u = UC(F = body.F)
    Target(x, u)
  }

  def measurements(x: X, v: Z): Z = ZC(dis = x.d) fuzz v

  private val K =
    UC.columns(
      F = XC.row(
        d = Matrix22.identity * linearDynamics.kd,
        v = Matrix22.identity * linearDynamics.kv
      )
    )

  def regulate(x: X, target: Target): U = {
    val xe: X = x - target.x
    val ue: U = U(K ** xe.vectorD)
    U(target.u.vectorD - ue.vectorD)
  }
}
