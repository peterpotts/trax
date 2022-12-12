package com.peterpotts.trax.math

/**
  * @author Peter Potts
  */
case class Matrix43(w: Vector3, x: Vector3, y: Vector3, z: Vector3) extends MatrixLike {
  lazy val matrixD: MatrixD = Vector(w.vectorD, x.vectorD, y.vectorD, z.vectorD)

  def ===(that: Matrix43): Boolean = w === that.w && x === that.x && y === that.y && z === that.z

  def fold(that: Matrix43)(f: (Vector3, Vector3) => Vector3) =
    Matrix43(f(w, that.w), f(x, that.x), f(y, that.y), f(z, that.z))

  def map1(f: Vector3 => Double): Vector4 = Vector4(f(w), f(x), f(y), f(z))

  def map3(f: Vector3 => Vector3): Matrix43 = Matrix43(f(w), f(x), f(y), f(z))

  def map4(f: Vector3 => Vector4): Matrix44 = Matrix44(f(w), f(x), f(y), f(z))

  def +(that: Matrix43): Matrix43 = fold(that)(_ + _)

  def -(that: Matrix43): Matrix43 = fold(that)(_ - _)

  def *(scalar: Double): Matrix43 = map3(_ * scalar)

  def *(vector3: Vector3): Vector4 = map1(_ dotProduct vector3)

  def *(that: Matrix33): Matrix43 =
    map3(row => Vector3(
      row.x * that.x.x + row.y * that.y.x + row.z * that.z.x,
      row.x * that.x.y + row.y * that.y.y + row.z * that.z.y,
      row.x * that.x.z + row.y * that.y.z + row.z * that.z.z
    ))

  def *(that: Matrix34): Matrix44 =
    map4(row => Vector4(
      row.x * that.x.w + row.y * that.y.w + row.z * that.z.w,
      row.x * that.x.x + row.y * that.y.x + row.z * that.z.x,
      row.x * that.x.y + row.y * that.y.y + row.z * that.z.y,
      row.x * that.x.z + row.y * that.y.z + row.z * that.z.z
    ))

  def /(scalar: Double): Matrix43 = map3(_ / scalar)

  lazy val transpose: Matrix34 = Matrix34(
    Vector4(w.x, x.x, y.x, z.x),
    Vector4(w.y, x.y, y.y, z.y),
    Vector4(w.z, x.z, y.z, z.z)
  )

  lazy val negate: Matrix43 = map3(_.negate)
}

object Matrix43 {
  val zero: Matrix43 = constant(0)

  def constant(x: Double): Matrix43 = Matrix43(
    Vector3.constant(x),
    Vector3.constant(x),
    Vector3.constant(x),
    Vector3.constant(x)
  )
}
