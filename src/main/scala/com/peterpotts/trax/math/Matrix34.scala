package com.peterpotts.trax.math

/**
  * @author Peter Potts
  */
case class Matrix34(x: Vector4, y: Vector4, z: Vector4) extends MatrixLike {
  lazy val matrixD: MatrixD = Vector(x.vectorD, y.vectorD, z.vectorD)

  def ===(that: Matrix34): Boolean = x === that.x && y === that.y && z === that.z

  def fold(that: Matrix34)(f: (Vector4, Vector4) => Vector4) = Matrix34(f(x, that.x), f(y, that.y), f(z, that.z))

  def map1(f: Vector4 => Double): Vector3 = Vector3(f(x), f(y), f(z))

  def map3(f: Vector4 => Vector3): Matrix33 = Matrix33(f(x), f(y), f(z))

  def map33(f: Vector4 => Matrix33): Tensor333 = Tensor333(f(x), f(y), f(z))

  def map4(f: Vector4 => Vector4): Matrix34 = Matrix34(f(x), f(y), f(z))

  def +(that: Matrix34): Matrix34 = fold(that)(_ + _)

  def -(that: Matrix34): Matrix34 = fold(that)(_ - _)

  def *(scalar: Double): Matrix34 = map4(_ * scalar)

  def *(vector4: Vector4): Vector3 = map1(_ dotProduct vector4)

  def *(that: Matrix43): Matrix33 =
    map3(row => Vector3(
      row.w * that.w.x + row.x * that.x.x + row.y * that.y.x + row.z * that.z.x,
      row.w * that.w.y + row.x * that.x.y + row.y * that.y.y + row.z * that.z.y,
      row.w * that.w.z + row.x * that.x.z + row.y * that.y.z + row.z * that.z.z
    ))

  def *(that: Matrix44): Matrix34 =
    map4(row => Vector4(
      row.w * that.w.w + row.x * that.x.w + row.y * that.y.w + row.z * that.z.w,
      row.w * that.w.x + row.x * that.x.x + row.y * that.y.x + row.z * that.z.x,
      row.w * that.w.y + row.x * that.x.y + row.y * that.y.y + row.z * that.z.y,
      row.w * that.w.z + row.x * that.x.z + row.y * that.y.z + row.z * that.z.z
    ))

  def *(quaternionMatrix: QuaternionMatrix): Matrix34 = *(quaternionMatrix.matrix44)

  def /(scalar: Double): Matrix34 = map4(_ / scalar)

  lazy val transpose: Matrix43 = Matrix43(
    Vector3(x.w, y.w, z.w),
    Vector3(x.x, y.x, z.x),
    Vector3(x.y, y.y, z.y),
    Vector3(x.z, y.z, z.z)
  )

  lazy val negate: Matrix34 = map4(_.negate)
}

object Matrix34 {
  val zero: Matrix34 = constant(0)

  def constant(x: Double): Matrix34 = Matrix34(
    Vector4.constant(x),
    Vector4.constant(x),
    Vector4.constant(x)
  )
}
