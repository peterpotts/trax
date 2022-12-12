package com.peterpotts.trax.math

/**
  * @author Peter Potts
  */
case class Tensor334(x: Matrix34, y: Matrix34, z: Matrix34) extends TensorLike {
  lazy val tensorD: TensorD = Vector(x.matrixD, y.matrixD, z.matrixD)

  def ===(that: Tensor334): Boolean = x === that.x && y === that.y && z === that.z

  def fold(that: Tensor334)(f: (Matrix34, Matrix34) => Matrix34) = Tensor334(f(x, that.x), f(y, that.y), f(z, that.z))

  def map3(f: Matrix34 => Vector3): Matrix33 = Matrix33(f(x), f(y), f(z))

  def map4(f: Matrix34 => Vector4): Matrix34 = Matrix34(f(x), f(y), f(z))

  def map33(f: Matrix34 => Matrix33): Tensor333 = Tensor333(f(x), f(y), f(z))

  def map34(f: Matrix34 => Matrix34): Tensor334 = Tensor334(f(x), f(y), f(z))

  def map43(f: Matrix34 => Matrix43): Tensor343 = Tensor343(f(x), f(y), f(z))

  def +(that: Tensor334): Tensor334 = fold(that)(_ + _)

  def -(that: Tensor334): Tensor334 = fold(that)(_ - _)

  def *(scalar: Double): Tensor334 = map34(_ * scalar)

  def *(vector4: Vector4): Matrix33 = map3(_ * vector4)

  def *(matrix43: Matrix43): Tensor333 = map33(_ * matrix43)

  def /(scalar: Double): Tensor334 = map34(_ / scalar)

  lazy val transposeIJ: Tensor334 = Tensor334(
    Matrix34(x.x, y.x, z.x),
    Matrix34(x.y, y.y, z.y),
    Matrix34(x.z, y.z, z.z)
  )

  lazy val transposeJK: Tensor343 = map43(_.transpose)
  lazy val transposeIK: Tensor433 = transposeIJ.transposeJK.transposeIJ
}

object Tensor334 {
  val zero: Tensor334 = constant(0)

  def constant(x: Double): Tensor334 = Tensor334(Matrix34.constant(x), Matrix34.constant(x), Matrix34.constant(x))

  case class IJK(ijk: Tensor334) {
    def ikj: Tensor343 = ijk.transposeJK

    def jki: Tensor433 = ikj.transposeIJ

    def kij: Tensor343 = ijk.transposeIJ.transposeJK
  }

}
