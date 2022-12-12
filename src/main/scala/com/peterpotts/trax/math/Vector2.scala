package com.peterpotts.trax.math

import com.peterpotts.trax.math.MathDecorator._

/**
  * @author Peter Potts
  */
case class Vector2(x: Double, y: Double) extends VectorLike {
  require(!x.isNaN, "x must be a number")
  require(!y.isNaN, "y must be a number")
  lazy val vectorD: VectorD = Vector(x, y)
  lazy val norm: Double = squareRoot(x.squared + y.squared)
  lazy val versor: Vector2 = this / norm

  def ===(that: Vector2): Boolean = x === that.x && y === that.y

  def fold(that: Vector2)(f: (Double, Double) => Double): Vector2 = Vector2(f(x, that.x), f(y, that.y))

  def map1(f: Double => Double): Vector2 = Vector2(f(x), f(y))

  def map2(f: Double => Vector2): Matrix22 = Matrix22(f(x), f(y))

  def distance(that: Vector2): Double = (this - that).norm

  def +(that: Vector2): Vector2 = fold(that)(_ + _)

  def -(that: Vector2): Vector2 = fold(that)(_ - _)

  def *(scalar: Double): Vector2 = map1(_ * scalar)

  def /(scalar: Double): Vector2 = map1(_ / scalar)

  def dotProduct(that: Vector2): Double = x * that.x + y * that.y

  lazy val negate: Vector2 = Vector2(-x, -y)

  lazy val transpose: Matrix12 = Matrix12(this)
}

object Vector2 {
  val zero: Vector2 = constant(0)
  val X: Vector2 = Vector2(1, 0)
  val Y: Vector2 = Vector2(0, 1)

  def constant(x: Double): Vector2 = Vector2(x, x)
}
