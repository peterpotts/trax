package com.peterpotts.trax.math

/**
  * @author Peter Potts
  */
case class Tensor333(x: Matrix33, y: Matrix33, z: Matrix33) extends TensorLike {
  lazy val tensorD: TensorD = Vector(x.matrixD, y.matrixD, z.matrixD)

  def ===(that: Tensor333): Boolean = x === that.x && y === that.y && z === that.z

  def fold(that: Tensor333)(f: (Matrix33, Matrix33) => Matrix33) = Tensor333(f(x, that.x), f(y, that.y), f(z, that.z))

  def map33(f: Matrix33 => Matrix33): Tensor333 = Tensor333(f(x), f(y), f(z))

  def map3(f: Matrix33 => Vector3): Matrix33 = Matrix33(f(x), f(y), f(z))

  def +(that: Tensor333): Tensor333 = fold(that)(_ + _)

  def -(that: Tensor333): Tensor333 = fold(that)(_ - _)

  def *(scalar: Double): Tensor333 = map33(_ * scalar)

  def *(vector3: Vector3): Matrix33 = map3(_ * vector3)

  def /(scalar: Double): Tensor333 = map33(_ / scalar)

  lazy val transposeIJ: Tensor333 = Tensor333(
    Matrix33(x.x, y.x, z.x),
    Matrix33(x.y, y.y, z.y),
    Matrix33(x.z, y.z, z.z)
  )

  lazy val transposeJK: Tensor333 = map33(_.transpose)
  lazy val transposeIK: Tensor333 = transposeIJ.transposeJK.transposeIJ
}

object Tensor333 {
  val zero: Tensor333 = constant(0)

  def constant(x: Double): Tensor333 = Tensor333(Matrix33.constant(x), Matrix33.constant(x), Matrix33.constant(x))

  case class IJK(ijk: Tensor333) {
    def ikj: Tensor333 = ijk.transposeJK

    def jki: Tensor333 = ikj.transposeIJ

    def kij: Tensor333 = ijk.transposeIJ.transposeJK
  }

  case class JKI(jki: Tensor333) {
    def ikj: Tensor333 = jki.transposeIJ

    def ijk: Tensor333 = ikj.transposeJK

    def kij: Tensor333 = ijk.transposeIJ.transposeJK
  }

}
