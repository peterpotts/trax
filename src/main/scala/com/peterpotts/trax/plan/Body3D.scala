package com.peterpotts.trax.plan

import com.peterpotts.trax.math._

/**
  * @author Peter Potts
  */
case class Body3D(
  mass: Double,
  d: Vector3,
  v: Vector3,
  q: UnitQuaternion,
  FB: Vector3,
  inertiaB: DiagonalMatrix33,
  omegaB: Vector3,
  tauB: Vector3
) {
  ////////////
  // Linear //
  ////////////

  lazy val a: Vector3 = F / mass
  lazy val p: Vector3 = v * mass
  lazy val F: Vector3 = RT * FB

  /////////////
  // Angular //
  /////////////

  def R: Matrix33 = q.rotationMatrix.matrix33

  def RT: Matrix33 = q.rotationMatrix.inverse.matrix33

  def e: EulerAngles = q.eulerAngles

  def r: RotationVector = q.rotationVector

  lazy val omega: Vector3 = RT * omegaB
  lazy val alphaB: Vector3 = inertiaB.inverse * (tauB - omegaB × (inertiaB * omegaB))
  lazy val alpha: Vector3 = RT * alphaB
  lazy val LB: Vector3 = inertiaB * omegaB
  lazy val L: Vector3 = RT * LB
  lazy val inertia: Matrix33 = RT * inertiaB * R
  lazy val inertiaInverse: Matrix33 = RT * inertiaB.inverse * R
  lazy val tau: Vector3 = RT * tauB
}

object Body3D {
  def apply(mass: Double, inertiaB: DiagonalMatrix33): Body3D = Body3D(
    mass = mass,
    d = Vector3.zero,
    v = Vector3.zero,
    q = UnitQuaternion.zero,
    FB = Vector3.zero,
    inertiaB = inertiaB,
    omegaB = Vector3.zero,
    tauB = Vector3.zero
  )
}
