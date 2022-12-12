package com.peterpotts.trax.models.i

import com.peterpotts.trax.kalman._
import com.peterpotts.trax.lqr._
import com.peterpotts.trax.math.VectorDDecorator._
import com.peterpotts.trax.math._
import com.peterpotts.trax.plan._

/**
  * @author Peter Potts
  */
case class ControllerI(mass: Double, inertiaB: DiagonalMatrix33, north: Vector3, dt: Double) extends Controller {
  private val planner = JourneyPlanner3D.example1(mass, inertiaB, dt)
  private val linearDynamics = LinearDynamics(mass, dt)
  private val angularDynamics = RotationVectorDynamics(inertiaB, dt)

  val initial: Target = toTarget(planner.initial)

  def target(time: Double, x: X, dt: Double): Target = toTarget(planner.plan(time, x, dt))

  private def toTarget(body: Body3D): Target = {
    val x = XI(
      d = body.d,
      v = body.v,
      a = body.a,
      r = body.r,
      omegaB = body.omegaB,
      alphaB = body.alphaB
    )

    val u = UI(
      FB = body.FB,
      tauB = body.tauB
    )

    Target(x, u)
  }

  def measurements(x: X, v: Z): Z = {
    val R = x.r.rotationMatrix.matrix33

    ZI(
      dis = x.d,
      vel = x.v,
      accB = R * x.a,
      magB = R * north,
      optB = x.omegaB,
      gyrB = x.alphaB
    ) fuzz v
  }

  def regulate(x: X, target: Target): U = {
    val reference = ReferenceI(x.r.unitQuaternion)
    val xr = reference.inverse.lambda(x)
    val tr = Target(reference.inverse.lambda(target.x), reference.inverse.mu(target.u))
    val ur = simpleRegulate(xr, tr)
    reference.mu(ur)
  }

  private def simpleRegulate(x: X, target: Target): U = {
    //val angularGain = angularDynamics.gain(x.r, x.omegaB)
    val angularGain = angularDynamics.Gain.zero
    val R = x.r.rotationMatrix.matrix33
    val xe = x - target.x

    val ue = UI(
      FB = R * (xe.d * linearDynamics.kd + xe.v * linearDynamics.kv),
      tauB = angularGain.kr * xe.r.vector3 + angularGain.ko * xe.omegaB
    )

    U(target.u.vectorD - ue.vectorD)
  }
}
