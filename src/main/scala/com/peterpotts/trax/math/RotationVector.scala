package com.peterpotts.trax.math

import com.peterpotts.trax.math.MathDecorator._

import scala.math._

/**
  * @author Peter Potts
  */
case class RotationVector(vector3: Vector3, validate: Boolean) {
  if (validate) {
    require(vector3.norm >= 0, s"norm ${vector3.norm} must be greater than or equal to 0")
    require(vector3.norm <= Pi, s"norm ${vector3.norm} must be less than or equal to π")
  }

  import vector3.{x, y, z}

  lazy val isLimit: Boolean = vector3.norm < RotationVector.lowerBound
  lazy val inverse: RotationVector = RotationVector(vector3.negate)

  def ===(that: RotationVector): Boolean = vector3 === that.vector3

  def *(other: RotationVector): RotationVector = (unitQuaternion ∘ other.unitQuaternion).rotationVector

  ////////////////
  // AXIS ANGLE //
  ////////////////

  /**
    * Page 18 (205, 206)
    * Mathematica (nv, αv)
    *
    * {{{
    * n_v(v)
    * α_v(v)
    * }}}
    */
  lazy val axisAngle: AxisAngle =
    if (vector3.norm === 0) AxisAngle.zero else AxisAngle(vector3.versor, vector3.norm)

  /////////////////////
  // ROTATION MATRIX //
  /////////////////////

  def rotationMatrix: RotationMatrix33 = axisAngle.rotationMatrix

  /**
    * Jacobian of rotation matrix with respect to rotation vector.
    *
    * {{{
    * dR_v{ij}         dR_q{ij}   dq_v{l}
    * -------- = Sigma -------- * -------
    *  dv{k}       l    dq{l}      dv{k}
    * }}}
    */
  lazy val jacobianRotationMatrix: Tensor333.IJK =
    Tensor333.IJK(unitQuaternion.jacobianRotationMatrix.ijk * jacobianUnitQuaternion)

  /////////////////////
  // UNIT QUATERNION //
  /////////////////////

  /**
    * Page 19 (208, 209)
    * Mathematica (qv)
    *
    * {{{
    * q_v(v)
    * }}}
    */
  lazy val unitQuaternion: UnitQuaternion = if (isLimit) limitUnitQuaternion else axisAngle.unitQuaternion

  lazy val limitUnitQuaternion: UnitQuaternion =
    Quaternion(w = 1, x = vector3.x / 2, y = vector3.y / 2, z = vector3.z / 2).versor

  /**
    * Jacobian of unit quaternion with respect to rotation vector.
    * Page 19 (210, 214, 215)
    * Mathematica (dqvdv)
    *
    * {{{
    *        dq_v{i}
    * G(v) = -------
    *         dv{j}
    * }}}
    */
  lazy val jacobianUnitQuaternion: Matrix43 = if (isLimit) limitJacobianUnitQuaternion else unsafeJacobianUnitQuaternion

  lazy val unsafeJacobianUnitQuaternion: Matrix43 = {
    val α = axisAngle.α
    val α2 = α.squared
    val shα = axisAngle.shα
    val chα = axisAngle.chα
    val a = chα * α - 2 * shα

    Matrix43(
      Vector3(-shα * x * α2, -shα * y * α2, -shα * z * α2),
      Vector3(a * x.squared + 2 * shα * α2, a * x * y, a * x * z),
      Vector3(a * x * y, a * y.squared + 2 * shα * α2, a * y * z),
      Vector3(a * x * z, a * y * z, a * z.squared + 2 * shα * α2)
    ) / (2 * α.cubed)
  }

  /**
    * Page 19 (215)
    * Mathematica (Ldqvdv)
    */
  lazy val limitJacobianUnitQuaternion: Matrix43 =
    Matrix43(
      Vector3(-x, -y, -z),
      Vector3(2, 0, 0),
      Vector3(0, 2, 0),
      Vector3(0, 0, 2)
    ) / 4

  ///////////////////////
  // QUATERNION MATRIX //
  ///////////////////////

  def quaternionMatrix: QuaternionMatrix = unitQuaternion.quaternionMatrix

  def conjugateQuaternionMatrix: QuaternionMatrix = unitQuaternion.conjugateQuaternionMatrix

  override def toString: String = List(vector3).mkString("RotationVector(", ", ", ")")
}

object RotationVector {
  val zero: RotationVector = RotationVector(Vector3.zero, validate = true)
  val lowerBound: Double = AxisAngle.lowerBound

  def normalize(vector3: Vector3): RotationVector = {
    val α = Angle.normalizePlus(vector3.norm)
    if (α == 0) zero else RotationVector(vector3.versor * α)
  }

  def apply(vector3: Vector3): RotationVector =
    if (vector3.norm === 0)
      zero
    else if (vector3.norm <= Pi)
      RotationVector(vector3, validate = true)
    else
      RotationVector(vector3.versor.negate * (TwoPi - vector3.norm), validate = true)
}
