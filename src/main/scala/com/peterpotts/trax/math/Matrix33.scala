package com.peterpotts.trax.math

import com.peterpotts.trax.math.Vector3._

/**
  * @author Peter Potts
  */
case class Matrix33(x: Vector3, y: Vector3, z: Vector3) extends MatrixLike {
  lazy val matrixD: MatrixD = Vector(x.vectorD, y.vectorD, z.vectorD)

  def ===(that: Matrix33): Boolean = x === that.x && y === that.y && z === that.z

  def fold(that: Matrix33)(f: (Vector3, Vector3) => Vector3) = Matrix33(f(x, that.x), f(y, that.y), f(z, that.z))

  def map1(f: Vector3 => Double): Vector3 = Vector3(f(x), f(y), f(z))

  def map3(f: Vector3 => Vector3): Matrix33 = Matrix33(f(x), f(y), f(z))

  def map4(f: Vector3 => Vector4): Matrix34 = Matrix34(f(x), f(y), f(z))

  def +(that: Matrix33): Matrix33 = fold(that)(_ + _)

  def -(that: Matrix33): Matrix33 = fold(that)(_ - _)

  def *(scalar: Double): Matrix33 = map3(_ * scalar)

  def *(vector1: Vector1): Matrix33 = *(vector1.value)

  def *(vector3: Vector3): Vector3 = map1(_ dotProduct vector3)

  def *(diagonalMatrix33: DiagonalMatrix33): Matrix33 = *(diagonalMatrix33.matrix33)

  def *(that: Matrix33): Matrix33 =
    map3(row => Vector3(
      row.x * that.x.x + row.y * that.y.x + row.z * that.z.x,
      row.x * that.x.y + row.y * that.y.y + row.z * that.z.y,
      row.x * that.x.z + row.y * that.y.z + row.z * that.z.z
    ))

  def *(that: Matrix34): Matrix34 =
    map4(row => Vector4(
      row.x * that.x.w + row.y * that.y.w + row.z * that.z.w,
      row.x * that.x.x + row.y * that.y.x + row.z * that.z.x,
      row.x * that.x.y + row.y * that.y.y + row.z * that.z.y,
      row.x * that.x.z + row.y * that.y.z + row.z * that.z.z
    ))

  def /(scalar: Double): Matrix33 = map3(_ / scalar)

  def /(vector1: Vector1): Matrix33 = /(vector1.value)

  lazy val cxx: Double = y.y * z.z - y.z * z.y
  lazy val cxy: Double = y.z * z.x - y.x * z.z
  lazy val cxz: Double = y.x * z.y - y.y * z.x
  lazy val cyx: Double = z.y * x.z - z.z * x.y
  lazy val cyy: Double = z.z * x.x - z.x * x.z
  lazy val cyz: Double = z.x * x.y - z.y * x.x
  lazy val czx: Double = x.y * y.z - x.z * y.y
  lazy val czy: Double = x.z * y.x - x.x * y.z
  lazy val czz: Double = x.x * y.y - x.y * y.x

  lazy val determinant: Double = x.x * cxx + x.y * cxy + x.z * cxz

  lazy val transpose: Matrix33 = Matrix33(
    Vector3(x.x, y.x, z.x),
    Vector3(x.y, y.y, z.y),
    Vector3(x.z, y.z, z.z)
  )

  lazy val adjugate: Matrix33 = Matrix33(
    Vector3(cxx, cyx, czx),
    Vector3(cxy, cyy, czy),
    Vector3(cxz, cyz, czz)
  )

  lazy val inverse: Matrix33 = adjugate / determinant
  lazy val negate: Matrix33 = map3(_.negate)
}

object Matrix33 {
  val zero: Matrix33 = constant(0)
  val identity = Matrix33(X, Y, Z)

  def constant(x: Double): Matrix33 = Matrix33(Vector3.constant(x), Vector3.constant(x), Vector3.constant(x))

  def diagonal(vector3: Vector3): Matrix33 =
    Matrix33(Vector3(vector3.x, 0, 0), Vector3(0, vector3.y, 0), Vector3(0, 0, vector3.z))
}
