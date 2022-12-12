package com.peterpotts.trax.models.k

import com.peterpotts.trax.kalman._
import com.peterpotts.trax.math._

case class ReferenceK(p: UnitQuaternion) extends Reference {
  private val RT = p.rotationMatrix.inverse.matrix33

  def lambda(xr: X): X = XK(
    d = RT * xr.d,
    v = RT * xr.v,
    a = RT * xr.a,
    e = (xr.e.unitQuaternion ∘ p).eulerAngles,
    omegaB = xr.omegaB,
    alphaB = xr.alphaB,
    g = RT * xr.g,
    N = RT * xr.N
  )

  def jacobianLambda(xr: X): MatrixD = XK.columns(
    d = XK.row(d = RT),
    v = XK.row(v = RT),
    a = XK.row(a = RT),
    e = XK.row(
      e = (xr.e.unitQuaternion ∘ p).jacobianEulerAngles * p.conjugateQuaternionMatrix * xr.e.jacobianUnitQuaternion
    ),
    omegaB = XK.row(omegaB = Matrix33.identity),
    alphaB = XK.row(alphaB = Matrix33.identity),
    g = XK.row(g = RT),
    N = XK.row(N = RT)
  )

  def mu(ur: U): U = ur

  def nu(zr: Z): Z = ZK(
    dis = RT * zr.dis,
    vel = RT * zr.vel,
    accB = zr.accB,
    magB = zr.magB,
    optB = zr.optB,
    gyrB = zr.gyrB
  )

  lazy val inverse: Reference = ReferenceK(p.inverse)
}
