package com.peterpotts.trax.math

/**
  * @author Peter Potts
  */
case class Matrix12(x: Vector2) extends MatrixLike {
  lazy val matrixD: MatrixD = Vector(x.vectorD)

  def ===(that: Matrix12): Boolean = x === that.x

  def fold(that: Matrix12)(f: (Vector2, Vector2) => Vector2) = Matrix12(f(x, that.x))

  def map1(f: Vector2 => Double): Vector1 = Vector1(f(x))

  def map2(f: Vector2 => Vector2): Matrix12 = Matrix12(f(x))

  def +(that: Matrix12): Matrix12 = fold(that)(_ + _)

  def -(that: Matrix12): Matrix12 = fold(that)(_ - _)

  def *(scalar: Double): Matrix12 = map2(_ * scalar)

  def *(vector2: Vector2): Vector1 = map1(_ dotProduct vector2)

  def *(that: Matrix22): Matrix12 = Matrix12(Vector2(x.x * that.x.x + x.y * that.y.x, x.x * that.x.y + x.y * that.y.y))

  def /(scalar: Double): Matrix12 = map2(_ / scalar)

  lazy val transpose: Vector2 = x
  lazy val negate: Matrix12 = map2(_.negate)
}

object Matrix12 {
  val zero: Matrix12 = constant(0)

  def constant(x: Double): Matrix12 = Matrix12(Vector2.constant(x))
}
