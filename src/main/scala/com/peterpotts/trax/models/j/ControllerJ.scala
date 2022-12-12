package com.peterpotts.trax.models.j

import com.peterpotts.trax.kalman._
import com.peterpotts.trax.lqr._
import com.peterpotts.trax.math.VectorDDecorator._
import com.peterpotts.trax.math._
import com.peterpotts.trax.plan._

/**
  * @author Peter Potts
  */
case class ControllerJ(mass: Double, inertiaB: DiagonalMatrix33, dt: Double) extends Controller {
  private val planner = JourneyPlanner3D.example1(mass, inertiaB, dt)
  private val linearDynamics = LinearDynamics(mass, dt)
  private val angularDynamics = RotationVectorDynamics(inertiaB, dt)

  val initial: Target = toTarget(planner.initial)

  def target(time: Double, x: X, dt: Double): Target = toTarget(planner.plan(time, x, dt))

  private def toTarget(body: Body3D): Target = {
    val x =
      XJ(
        d = body.d,
        v = body.v,
        a = body.a,
        r = body.r,
        omegaB = body.omegaB,
        alphaB = body.alphaB,
        g = Vector3.Z,
        N = Vector3.X
      )

    val u =
      UJ(
        FB = body.FB,
        tauB = body.tauB
      )

    Target(x, u)
  }

  def measurements(x: X, v: Z): Z = {
    val R = x.r.rotationMatrix.matrix33

    ZJ(
      dis = x.d,
      vel = x.v,
      accB = R * x.a,
      magB = R * x.N,
      optB = x.omegaB,
      gyrB = x.alphaB
    ) fuzz v
  }

  def regulate(x: X, target: Target): U = {
    val reference = ReferenceJ(x.r.unitQuaternion)
    val xr = reference.inverse.lambda(x)
    val tr = Target(reference.inverse.lambda(target.x), reference.inverse.mu(target.u))
    val ur = simpleRegulate(xr, tr)
    reference.mu(ur)
  }

  private def simpleRegulate(x: X, target: Target): U = {
    //val angularGain = angularDynamics.gain(x.r, x.omegaB)
    val angularGain = angularDynamics.Gain.zero
    val R = x.r.rotationMatrix.matrix33
    val xe: X = x - target.x

    val ue =
      UJ(
        FB = R * (xe.d * linearDynamics.kd + xe.v * linearDynamics.kv),
        tauB = angularGain.kr * xe.r.vector3 + angularGain.ko * xe.omegaB
      )

    U(target.u.vectorD - ue.vectorD)
  }
}
