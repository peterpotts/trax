package com.peterpotts.trax.models.h

import com.peterpotts.trax.kalman._
import com.peterpotts.trax.lqr._
import com.peterpotts.trax.math.VectorDDecorator._
import com.peterpotts.trax.math._
import com.peterpotts.trax.plan._

/**
  * @author Peter Potts
  */
case class ControllerH(mass: Double, inertiaB: DiagonalMatrix33, north: Vector3, dt: Double) extends Controller {
  private val planner = JourneyPlanner3D.example1(mass, inertiaB, dt)
  private val linearDynamics = LinearDynamics(mass, dt)

  /**
    * QuaternionDynamics cannot be used because it is not locally linear at the origin.
    */
  private val angularDynamics = RotationVectorDynamics(inertiaB, dt)

  val initial: Target = toTarget(planner.initial)

  def target(time: Double, x: X, dt: Double): Target = toTarget(planner.plan(time, x, dt))

  private def toTarget(body: Body3D): Target = {
    val x = XH(
      d = body.d,
      v = body.v,
      a = body.a,
      q = body.q,
      omegaB = body.omegaB,
      alphaB = body.alphaB
    )

    val u = UH(
      FB = body.FB,
      tauB = body.tauB
    )

    Target(x, u)
  }

  def measurements(x: X, v: Z): Z = {
    val R = x.q.rotationMatrix.matrix33

    ZH(
      dis = x.d,
      vel = x.v,
      accB = R * x.a,
      magB = R * north,
      optB = x.omegaB,
      gyrB = x.alphaB
    ) fuzz v
  }

  def regulate(x: X, target: Target): U = {
    val reference = ReferenceH(x.q)
    val xr = reference.inverse.lambda(x)
    val tr = Target(reference.inverse.lambda(target.x), reference.inverse.mu(target.u))
    val ur = simpleRegulate(xr, tr)
    reference.mu(ur)
  }

  private def simpleRegulate(x: X, target: Target): U = {
    //val angularGain = angularDynamics.gain(x.q.rotationVector, x.omegaB)
    val angularGain = angularDynamics.Gain.zero
    val R = x.q.rotationMatrix.matrix33
    val xe = x - target.x

    val ue = UH(
      FB = R * (xe.d * linearDynamics.kd + xe.v * linearDynamics.kv),
      tauB = angularGain.kr * xe.q.rotationVector.vector3 + angularGain.ko * xe.omegaB
    )

    U(target.u.vectorD - ue.vectorD)
  }
}
