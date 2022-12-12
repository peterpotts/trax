package com.peterpotts.trax.models.i

import com.peterpotts.trax.kalman._
import com.peterpotts.trax.math._

/**
  * Orientation controller.
  *
  * @param p Orientation of the reference coordinate system with respect the world coordinate system.
  * @author Peter Potts
  */
case class ReferenceI(p: UnitQuaternion) extends Reference {
  private val RT = p.rotationMatrix.inverse.matrix33

  def lambda(xr: X): X = XI(
    d = RT * xr.d,
    v = RT * xr.v,
    a = RT * xr.a,
    r = (xr.r.unitQuaternion ∘ p).rotationVector,
    omegaB = xr.omegaB,
    alphaB = xr.alphaB
  )

  def jacobianLambda(xr: X): MatrixD = XI.columns(
    d = XI.row(d = RT),
    v = XI.row(v = RT),
    a = XI.row(a = RT),
    r = XI.row(r = (xr.r.unitQuaternion ∘ p).jacobianRotationVector * p.conjugateQuaternionMatrix * xr.r.jacobianUnitQuaternion),
    omegaB = XI.row(omegaB = Matrix33.identity),
    alphaB = XI.row(alphaB = Matrix33.identity)
  )

  def mu(ur: U): U = ur

  def nu(zr: Z): Z = ZI(
    dis = RT * zr.dis,
    vel = RT * zr.vel,
    accB = zr.accB,
    magB = zr.magB,
    optB = zr.optB,
    gyrB = zr.gyrB
  )

  lazy val inverse: Reference = ReferenceI(p.inverse)
}
