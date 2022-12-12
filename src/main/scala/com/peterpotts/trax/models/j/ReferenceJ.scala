package com.peterpotts.trax.models.j

import com.peterpotts.trax.kalman._
import com.peterpotts.trax.math._

/**
  * Orientation controller.
  *
  * @param p Orientation of the reference coordinate system with respect the world coordinate system.
  *
  * @author Peter Potts
  */
case class ReferenceJ(p: UnitQuaternion) extends Reference {
  private val RT = p.rotationMatrix.inverse.matrix33

  def lambda(xr: X): X = XJ(
    d = RT * xr.d,
    v = RT * xr.v,
    a = RT * xr.a,
    r = (xr.r.unitQuaternion ∘ p).rotationVector,
    omegaB = xr.omegaB,
    alphaB = xr.alphaB,
    g = RT * xr.g,
    N = RT * xr.N
  )

  def jacobianLambda(xr: X): MatrixD = XJ.columns(
    d = XJ.row(d = RT),
    v = XJ.row(v = RT),
    a = XJ.row(a = RT),
    r = XJ.row(r = (xr.r.unitQuaternion ∘ p).jacobianRotationVector * p.conjugateQuaternionMatrix * xr.r.jacobianUnitQuaternion),
    omegaB = XJ.row(omegaB = Matrix33.identity),
    alphaB = XJ.row(alphaB = Matrix33.identity),
    g = XJ.row(g = RT),
    N = XJ.row(N = RT)
  )

  def mu(ur: U): U = ur

  def nu(zr: Z): Z = ZJ(
    dis = RT * zr.dis,
    vel = RT * zr.vel,
    accB = zr.accB,
    magB = zr.magB,
    optB = zr.optB,
    gyrB = zr.gyrB
  )

  lazy val inverse: Reference = ReferenceJ(p.inverse)
}
