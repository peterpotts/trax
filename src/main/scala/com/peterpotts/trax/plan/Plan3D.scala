package com.peterpotts.trax.plan

import com.peterpotts.trax.math.{RotationVector, Vector3}

/**
  * @author Peter Potts
  */
sealed trait Plan3D {
  def iterate(body: Body3D, dt: Double): Body3D
}

sealed trait BodyFixed3D extends Plan3D {
  val FB: Vector3
  val tauB: Vector3

  def iterate(body: Body3D, dt: Double): Body3D = {
    val R = body.R
    val RT = body.RT

    val F = RT * FB
    val p = body.p + F * dt
    val v = p / body.mass
    val d = body.d + v * dt

    val tau = RT * tauB
    val L = body.L + tau * dt
    val omega = body.inertiaInverse * L
    val omegaB = R * omega

    val q = body.q ∘ RotationVector(omega * dt).unitQuaternion

    body.copy(
      d = d,
      v = v,
      q = q,
      FB = FB,
      omegaB = omegaB,
      tauB = tauB
    )
  }
}

case class ForceTorque3D(FB: Vector3, tauB: Vector3) extends BodyFixed3D

object Cruise3D extends BodyFixed3D {
  val FB: Vector3 = Vector3.zero
  val tauB: Vector3 = Vector3.zero
}

case class Force3D(FB: Vector3) extends BodyFixed3D {
  val tauB: Vector3 = Vector3.zero
}

case class Torque3D(tauB: Vector3) extends BodyFixed3D {
  val FB: Vector3 = Vector3.zero
}
