package com.peterpotts.trax.math

import com.peterpotts.trax.math.MathDecorator._

/**
  * @author Peter Potts
  */
case class Vector4(w: Double, x: Double, y: Double, z: Double) extends VectorLike {
  require(!w.isNaN, "w must be a number")
  require(!x.isNaN, "x must be a number")
  require(!y.isNaN, "y must be a number")
  require(!z.isNaN, "z must be a number")
  lazy val vectorD: VectorD = Vector(w, x, y, z)

  def ===(that: Vector4): Boolean = w === that.w && x === that.x && y === that.y && z === that.z

  def fold(that: Vector4)(f: (Double, Double) => Double): Vector4 =
    Vector4(f(w, that.w), f(x, that.x), f(y, that.y), f(z, that.z))

  def map1(f: Double => Double): Vector4 = Vector4(f(w), f(x), f(y), f(z))

  def +(that: Vector4): Vector4 = fold(that)(_ + _)

  def -(that: Vector4): Vector4 = fold(that)(_ - _)

  def *(scalar: Double): Vector4 = map1(_ * scalar)

  def /(scalar: Double): Vector4 = map1(_ / scalar)

  def dotProduct(that: Vector4): Double = w * that.w + x * that.x + y * that.y + z * that.z

  lazy val negate: Vector4 = Vector4(-w, -x, -y, -z)
}

object Vector4 {
  val zero: Vector4 = constant(0)
  val W: Vector4 = Vector4(1, 0, 0, 0)
  val X: Vector4 = Vector4(0, 1, 0, 0)
  val Y: Vector4 = Vector4(0, 0, 1, 0)
  val Z: Vector4 = Vector4(0, 0, 0, 1)

  def constant(x: Double): Vector4 = Vector4(x, x, x, x)
}
