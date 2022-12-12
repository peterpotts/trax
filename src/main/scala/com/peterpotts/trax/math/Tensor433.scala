package com.peterpotts.trax.math

/**
  * @author Peter Potts
  */
case class Tensor433(w: Matrix33, x: Matrix33, y: Matrix33, z: Matrix33) extends TensorLike {
  lazy val tensorD: TensorD = Vector(w.matrixD, x.matrixD, y.matrixD, z.matrixD)

  def ===(that: Tensor433): Boolean = w == that.w && x === that.x && y === that.y && z === that.z

  def fold(that: Tensor433)(f: (Matrix33, Matrix33) => Matrix33) = Tensor433(f(w, that.w), f(x, that.x), f(y, that.y), f(z, that.z))

  def map3(f: Matrix33 => Vector3): Matrix43 = Matrix43(f(w), f(x), f(y), f(z))

  def map4(f: Matrix33 => Vector4): Matrix44 = Matrix44(f(w), f(x), f(y), f(z))

  def map33(f: Matrix33 => Matrix33): Tensor433 = Tensor433(f(w), f(x), f(y), f(z))

  def map34(f: Matrix33 => Matrix33): Tensor433 = Tensor433(f(w), f(x), f(y), f(z))

  def +(that: Tensor433): Tensor433 = fold(that)(_ + _)

  def -(that: Tensor433): Tensor433 = fold(that)(_ - _)

  def *(scalar: Double): Tensor433 = map34(_ * scalar)

  def *(vector3: Vector3): Matrix43 = map3(_ * vector3)

  def *(matrix33: Matrix33): Tensor433 = map33(_ * matrix33)

  def /(scalar: Double): Tensor433 = map34(_ / scalar)

  lazy val transposeIJ: Tensor343 = Tensor343(
    Matrix43(w.x, x.x, y.x, z.x),
    Matrix43(w.y, x.y, y.y, z.y),
    Matrix43(w.z, x.z, y.z, z.z)
  )

  lazy val transposeJK: Tensor433 = map33(_.transpose)
  lazy val transposeIK: Tensor334 = transposeIJ.transposeJK.transposeIJ
}

object Tensor433 {
  val zero: Tensor433 = constant(0)

  def constant(x: Double): Tensor433 =
    Tensor433(Matrix33.constant(x), Matrix33.constant(x), Matrix33.constant(x), Matrix33.constant(x))

  case class JKI(jki: Tensor433) {
    def ikj: Tensor343 = jki.transposeIJ

    def ijk: Tensor334 = ikj.transposeJK

    def kij: Tensor343 = ijk.transposeIJ.transposeJK
  }

}
