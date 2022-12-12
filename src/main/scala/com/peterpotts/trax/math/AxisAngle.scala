package com.peterpotts.trax.math

import com.peterpotts.trax.math.MathDecorator._

import scala.math._

/**
  * @author Peter Potts
  */
case class AxisAngle(n: Vector3, α: Double, validate: Boolean) {
  if (validate) {
    require(n.norm === 1, s"axis norm ${n.norm} must be one")
    require(α >= 0, s"angle $α must be greater than or equal to zero")
    require(α <= Pi, s"angle $α must be less than or equal to π")
  }

  lazy val shα: Double = sin(α / 2)
  lazy val chα: Double = cos(α / 2)
  lazy val isLimit: Boolean = α < AxisAngle.lowerBound
  lazy val inverse: AxisAngle = AxisAngle(n, TwoPi - α)

  def ===(that: AxisAngle): Boolean = n === that.n && α === that.α

  /**
    * Page 18 (204)
    *
    * | v_a(α, n)
    */
  lazy val rotationVector: RotationVector = RotationVector(n * α)

  /**
    * Page 17 (175)
    *
    * | q_a(α, n)
    */
  lazy val unitQuaternion: UnitQuaternion =
    UnitQuaternion(Quaternion(w = chα, x = n.x * shα, y = n.y * shα, z = n.z * shα))

  /**
    * Page 17 (183)
    *
    * | R_a(α, n)
    */
  lazy val rotationMatrix: RotationMatrix33 = unitQuaternion.rotationMatrix

  @transient lazy val display: String = s"AxisAngle($n, ${α.toDegrees})"

  override def toString: String = List(n, α).mkString("AxisAngle(", ", ", ")")
}

object AxisAngle {
  val zero: AxisAngle = AxisAngle(n = Vector3.X, α = 0, validate = true)
  val limit = 0.02
  val lowerBound: Double = limit

  def normalize(n: Vector3, α: Double): AxisAngle = AxisAngle(n, Angle.normalizePlus(α))

  def apply(n: Vector3, α: Double): AxisAngle =
    if (α <= Pi)
      AxisAngle(n, α, validate = true)
    else
      AxisAngle(n.negate, TwoPi - α, validate = true)
}
