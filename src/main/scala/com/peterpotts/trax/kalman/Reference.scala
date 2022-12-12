package com.peterpotts.trax.kalman

import com.peterpotts.trax.math._

/**
  * @author Peter Potts
  */
trait Reference {
  val p: UnitQuaternion

  def lambda(xr: X): X

  def jacobianLambda(xr: X): MatrixD

  def mu(ur: U): U

  def nu(zr: Z): Z

  val inverse: Reference
}

object Reference {

  object identity extends Reference {
    val p: UnitQuaternion = UnitQuaternion.zero

    def lambda(xr: X): X = xr

    def jacobianLambda(xr: X): MatrixD = MatrixD.identity(xr.vectorD.size)

    def mu(ur: U): U = ur

    def nu(zr: Z): Z = zr

    lazy val inverse: Reference = this
  }

}
