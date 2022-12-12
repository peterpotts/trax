package com.peterpotts.trax.models.h

import com.peterpotts.trax.kalman._
import com.peterpotts.trax.math._

/**
  * Orientation controller.
  *
  * @param p Orientation of the reference coordinate system with respect the world coordinate system.
  * @author Peter Potts
  */
case class ReferenceH(p: UnitQuaternion) extends Reference {
  private val RT = p.rotationMatrix.inverse.matrix33

  def lambda(xr: X): X = XH(
    d = RT * xr.d,
    v = RT * xr.v,
    a = RT * xr.a,
    q = xr.q ∘ p,
    omegaB = xr.omegaB,
    alphaB = xr.alphaB
  )

  def jacobianLambda(xr: X): MatrixD = XH.columns(
    d = XH.threeRows(d = RT),
    v = XH.threeRows(v = RT),
    a = XH.threeRows(a = RT),
    q = XH.fourRows(q = p.conjugateQuaternionMatrix.matrix44),
    omegaB = XH.threeRows(omegaB = Matrix33.identity),
    alphaB = XH.threeRows(alphaB = Matrix33.identity)
  )

  def mu(ur: U): U = ur

  def nu(zr: Z): Z = ZH(
    dis = RT * zr.dis,
    vel = RT * zr.vel,
    accB = zr.accB,
    magB = zr.magB,
    optB = zr.optB,
    gyrB = zr.gyrB
  )

  lazy val inverse: Reference = ReferenceH(p.inverse)
}
