package com.peterpotts.trax.math

/**
  * @author Peter Potts
  */
case class Tensor343(x: Matrix43, y: Matrix43, z: Matrix43) extends TensorLike {
  lazy val tensorD: TensorD = Vector(x.matrixD, y.matrixD, z.matrixD)

  def ===(that: Tensor343): Boolean = x === that.x && y === that.y && z === that.z

  def fold(that: Tensor343)(f: (Matrix43, Matrix43) => Matrix43) = Tensor343(f(x, that.x), f(y, that.y), f(z, that.z))

  def map3(f: Matrix43 => Vector3): Matrix33 = Matrix33(f(x), f(y), f(z))

  def map4(f: Matrix43 => Vector4): Matrix34 = Matrix34(f(x), f(y), f(z))

  def map33(f: Matrix43 => Matrix33): Tensor333 = Tensor333(f(x), f(y), f(z))

  def map34(f: Matrix43 => Matrix34): Tensor334 = Tensor334(f(x), f(y), f(z))

  def map43(f: Matrix43 => Matrix43): Tensor343 = Tensor343(f(x), f(y), f(z))

  def +(that: Tensor343): Tensor343 = fold(that)(_ + _)

  def -(that: Tensor343): Tensor343 = fold(that)(_ - _)

  def *(scalar: Double): Tensor343 = map43(_ * scalar)

  def *(vector3: Vector3): Matrix34 = map4(_ * vector3)

  def *(matrix33: Matrix33): Tensor343 = map43(_ * matrix33)

  def /(scalar: Double): Tensor343 = map43(_ / scalar)

  lazy val transposeIJ: Tensor433 = Tensor433(
    Matrix33(x.w, y.w, z.w),
    Matrix33(x.x, y.x, z.x),
    Matrix33(x.y, y.y, z.y),
    Matrix33(x.z, y.z, z.z)
  )

  lazy val transposeJK: Tensor334 = map34(_.transpose)
  lazy val transposeIK: Tensor343 = transposeIJ.transposeJK.transposeIJ
}

object Tensor343 {
  val zero: Tensor343 = constant(0)

  def constant(x: Double): Tensor343 = Tensor343(Matrix43.constant(x), Matrix43.constant(x), Matrix43.constant(x))
}
