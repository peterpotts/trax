package com.peterpotts.trax.math

import com.peterpotts.trax.math.MathDecorator._

import scala.math._

/**
  * Passive transformation rotation matrix. The active transformation rotation matrix is just the transpose. An active
  * transformation is used to change a position while a passive transformation is used to change a coordinate system.
  *
  * R123(ϕ, θ, ψ)
  *
  * @author Peter Potts
  */
case class RotationMatrix33(matrix33: Matrix33) {
  require(matrix33.determinant === 1, "matrix must have unit determinant")
  require((matrix33 * matrix33.transpose) === Matrix33.identity, "matrix transpose must be matrix inverse")

  def *(other: RotationMatrix33): RotationMatrix33 = RotationMatrix33(matrix33 * other.matrix33)

  lazy val inverse: RotationMatrix33 = RotationMatrix33(matrix33.transpose)

  /**
    * Page 24 (289)
    * u_123(R)
    */
  lazy val eulerAngles: EulerAngles = EulerAngles.normalize(
    arcTan2(matrix33.y.z, matrix33.z.z),
    -arcSin(matrix33.x.z),
    arcTan2(matrix33.x.y, matrix33.x.x)
  )
}

object RotationMatrix33 {
  /**
    * Page 5 (14)
    * R_1(ϕ)
    */
  def X(ϕ: Double): RotationMatrix33 = {
    val s = sin(ϕ)
    val c = cos(ϕ)

    RotationMatrix33(Matrix33(
      Vector3(1, 0, 0),
      Vector3(0, c, s),
      Vector3(0, -s, c)
    ))
  }

  /**
    * Page 5 (15)
    * R_2(θ)
    */
  def Y(θ: Double): RotationMatrix33 = {
    val s = sin(θ)
    val c = cos(θ)

    RotationMatrix33(Matrix33(
      Vector3(c, 0, -s),
      Vector3(0, 1, 0),
      Vector3(s, 0, c)
    ))
  }

  /**
    * Page 5 (16)
    * R_3(ψ)
    */
  def Z(ψ: Double): RotationMatrix33 = {
    val s = sin(ψ)
    val c = cos(ψ)

    RotationMatrix33(Matrix33(
      Vector3(c, s, 0),
      Vector3(-s, c, 0),
      Vector3(0, 0, 1)
    ))
  }
}
