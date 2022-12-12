package com.peterpotts.trax.math

import com.peterpotts.trax.math.Vector4._

/**
  * @author Peter Potts
  */
case class Matrix44(w: Vector4, x: Vector4, y: Vector4, z: Vector4) extends MatrixLike {
  lazy val matrixD: MatrixD = Vector(w.vectorD, x.vectorD, y.vectorD, z.vectorD)

  def ===(that: Matrix44): Boolean = w === that.w && x === that.x && y === that.y && z === that.z

  def fold(that: Matrix44)(f: (Vector4, Vector4) => Vector4) =
    Matrix44(f(w, that.w), f(x, that.x), f(y, that.y), f(z, that.z))

  def map1(f: Vector4 => Double): Vector4 = Vector4(f(w), f(x), f(y), f(z))

  def map3(f: Vector4 => Vector3): Matrix43 = Matrix43(f(w), f(x), f(y), f(z))

  def map4(f: Vector4 => Vector4): Matrix44 = Matrix44(f(w), f(x), f(y), f(z))

  def +(that: Matrix44): Matrix44 = fold(that)(_ + _)

  def -(that: Matrix44): Matrix44 = fold(that)(_ - _)

  def *(scalar: Double): Matrix44 = map4(_ * scalar)

  def *(vector4: Vector4): Vector4 = map1(_ dotProduct vector4)

  def *(that: Matrix43): Matrix43 =
    map3(row => Vector3(
      row.w * that.w.x + row.x * that.x.x + row.y * that.y.x + row.z * that.z.x,
      row.w * that.w.y + row.x * that.x.y + row.y * that.y.y + row.z * that.z.y,
      row.w * that.w.z + row.x * that.x.z + row.y * that.y.z + row.z * that.z.z
    ))


  def *(that: Matrix44): Matrix44 =
    map4(row => Vector4(
      row.w * that.w.w + row.x * that.x.w + row.y * that.y.w + row.z * that.z.w,
      row.w * that.w.x + row.x * that.x.x + row.y * that.y.x + row.z * that.z.x,
      row.w * that.w.y + row.x * that.x.y + row.y * that.y.y + row.z * that.z.y,
      row.w * that.w.z + row.x * that.x.z + row.y * that.y.z + row.z * that.z.z
    ))

  def /(scalar: Double): Matrix44 = map4(_ / scalar)

  lazy val transpose: Matrix44 = Matrix44(
    Vector4(w.w, x.w, y.w, z.w),
    Vector4(w.x, x.x, y.x, z.x),
    Vector4(w.y, x.y, y.y, z.y),
    Vector4(w.z, x.z, y.z, z.z)
  )

  lazy val negate: Matrix44 = map4(_.negate)
}

object Matrix44 {
  val zero: Matrix44 = constant(0)
  val identity = Matrix44(W, X, Y, Z)

  def constant(x: Double): Matrix44 = Matrix44(
    Vector4.constant(x),
    Vector4.constant(x),
    Vector4.constant(x),
    Vector4.constant(x)
  )

  def diagonal(vector4: Vector4): Matrix44 = Matrix44(
    Vector4(vector4.w, 0, 0, 0),
    Vector4(0, vector4.x, 0, 0),
    Vector4(0, 0, vector4.y, 0),
    Vector4(0, 0, 0, vector4.z)
  )
}
