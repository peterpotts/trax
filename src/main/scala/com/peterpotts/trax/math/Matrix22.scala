package com.peterpotts.trax.math

import com.peterpotts.trax.math.Vector2._

/**
  * @author Peter Potts
  */
case class Matrix22(x: Vector2, y: Vector2) extends MatrixLike {
  lazy val matrixD: MatrixD = Vector(x.vectorD, y.vectorD)

  def ===(that: Matrix22): Boolean = x === that.x && y === that.y

  def fold(that: Matrix22)(f: (Vector2, Vector2) => Vector2) = Matrix22(f(x, that.x), f(y, that.y))

  def map1(f: Vector2 => Double): Vector2 = Vector2(f(x), f(y))

  def map2(f: Vector2 => Vector2): Matrix22 = Matrix22(f(x), f(y))

  def +(that: Matrix22): Matrix22 = fold(that)(_ + _)

  def -(that: Matrix22): Matrix22 = fold(that)(_ - _)

  def *(scalar: Double): Matrix22 = map2(_ * scalar)

  def *(vector1: Vector1): Matrix22 = *(vector1.value)

  def *(vector2: Vector2): Vector2 = map1(_ dotProduct vector2)

  def *(that: Matrix22): Matrix22 =
    map2(row => Vector2(
      row.x * that.x.x + row.y * that.y.x,
      row.x * that.x.y + row.y * that.y.y
    ))

  def /(scalar: Double): Matrix22 = map2(_ / scalar)

  def /(vector1: Vector1): Matrix22 = /(vector1.value)

  lazy val cxx: Double = y.y
  lazy val cxy: Double = -y.x
  lazy val cyx: Double = -x.y
  lazy val cyy: Double = x.x

  lazy val determinant: Double = x.x * cxx + x.y * cxy

  lazy val transpose: Matrix22 = Matrix22(
    Vector2(x.x, y.x),
    Vector2(x.y, y.y)
  )

  lazy val adjugate: Matrix22 = Matrix22(
    Vector2(cxx, cyx),
    Vector2(cxy, cyy)
  )

  lazy val inverse: Matrix22 = adjugate / determinant
  lazy val negate: Matrix22 = map2(_.negate)
}

object Matrix22 {
  val zero: Matrix22 = constant(0)
  val identity = Matrix22(X, Y)

  def constant(x: Double): Matrix22 = Matrix22(Vector2.constant(x), Vector2.constant(x))

  def diagonal(vector2: Vector2): Matrix22 = Matrix22(Vector2(vector2.x, 0), Vector2(0, vector2.y))
}
