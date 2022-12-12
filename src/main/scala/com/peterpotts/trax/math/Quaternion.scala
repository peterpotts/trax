package com.peterpotts.trax.math

import com.peterpotts.trax.math.MathDecorator._

/**
  * Note that as the length tends to zero, the direction of the rotation axis becomes very inaccurate.
  *
  * @author Peter Potts
  */
case class Quaternion(w: Double, x: Double, y: Double, z: Double) {
  lazy val vector4: Vector4 = Vector4(w, x, y, z)
  lazy val scalar: Quaternion = Quaternion(w, 0, 0, 0)
  lazy val vector3: Vector3 = Vector3(x, y, z)
  lazy val norm: Double = squareRoot(w.squared + x.squared + y.squared + z.squared)
  lazy val length: Double = 1 - w.squared
  lazy val versor: UnitQuaternion = UnitQuaternion(this / norm)
  lazy val conjugate: Quaternion = Quaternion(w, -x, -y, -z)
  lazy val negate: Quaternion = Quaternion(-w, -x, -y, -z)
  lazy val isNegative: Boolean = w < 0
  lazy val abs: Quaternion = if (isNegative) negate else this
  lazy val inverse: Quaternion = conjugate / norm.squared

  def ===(that: Quaternion): Boolean = w === that.w && x === that.x && y === that.y && z === that.z

  /**
    * Page 14 (109)
    * Mathematica (Q)
    *
    * | Q(q)
    */
  lazy val quaternionMatrix: QuaternionMatrix = QuaternionMatrix(Matrix44(
    Vector4(w, -x, -y, -z),
    Vector4(x, w, z, -y),
    Vector4(y, -z, w, x),
    Vector4(z, y, -x, w)
  ))

  /**
    * Page 14 (111)
    * Mathematica (cQ)
    *
    * | Q'(q)
    */
  lazy val conjugateQuaternionMatrix: QuaternionMatrix = QuaternionMatrix(Matrix44(
    Vector4(w, -x, -y, -z),
    Vector4(x, w, -z, y),
    Vector4(y, z, w, -x),
    Vector4(z, -y, x, w)
  ))

  def +(scalar: Double): Quaternion = Quaternion(w + scalar, x + scalar, y + scalar, z + scalar)

  def -(scalar: Double): Quaternion = Quaternion(w - scalar, x - scalar, y - scalar, z - scalar)

  def *(scalar: Double): Quaternion = Quaternion(w * scalar, x * scalar, y * scalar, z * scalar)

  def /(scalar: Double): Quaternion = Quaternion(w / scalar, x / scalar, y / scalar, z / scalar)

  def +(that: Quaternion): Quaternion = Quaternion(w + that.w, x + that.x, y + that.y, z + that.z)

  def -(that: Quaternion): Quaternion = Quaternion(w - that.w, x - that.x, y - that.y, z - that.z)

  /**
    * Derived from Page 14 (103)
    * Mathematica (qm)
    *
    * | q_m(q, p)
    */
  def hamiltonProduct(that: Quaternion): Quaternion = Quaternion(
    w = w * that.w - x * that.x - y * that.y - z * that.z,
    x = w * that.x + x * that.w - y * that.z + z * that.y,
    y = w * that.y + x * that.z + y * that.w - z * that.x,
    z = w * that.z - x * that.y + y * that.x + z * that.w
  )

  def ∘(that: Quaternion): Quaternion = hamiltonProduct(that)
}

object Quaternion {
  val zero: Quaternion = Quaternion(0, 0, 0, 0)
  val identity: Quaternion = UnitQuaternion.zero.quaternion

  def apply(vector4: Vector4): Quaternion = Quaternion(vector4.w, vector4.x, vector4.y, vector4.z)
}
