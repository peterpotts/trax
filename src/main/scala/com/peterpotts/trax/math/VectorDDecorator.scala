package com.peterpotts.trax.math

import com.peterpotts.trax.math.MathDecorator._

import scala.util.Random

/**
  * @author Peter Potts
  */
object VectorDDecorator {

  implicit class DecoratedVectorD(val vectorD: VectorD) extends AnyVal {
    def is(rows: Int): Boolean = vectorD.size == rows

    def rows: Vector[Int] = vectorD.indices.toVector

    def ===(other: VectorD): Boolean =
      vectorD.size == other.size && rows.forall(index => vectorD(index) === other(index))

    def matrixD: MatrixD = vectorD.map(VectorD(_))

    def vector1: Vector1 = {
      require(vectorD.is(1), "vectorD must be 1 vector")
      Vector1(vectorD.x)
    }

    def vector2: Vector2 = {
      require(vectorD.is(2), "vectorD must be 2 vector")
      Vector2(vectorD.x, vectorD.y)
    }

    def vector3: Vector3 = {
      require(vectorD.is(3), "vectorD must be 3 vector")
      Vector3(vectorD.x, vectorD.y, vectorD.z)
    }

    def vector4: Vector4 = {
      require(vectorD.is(4), "vectorD must be 4 vector")
      Vector4(vectorD(0), vectorD(1), vectorD(2), vectorD(3))
    }

    def squared: VectorD = vectorD.map(_.squared)

    def normSquared: Double = squared.sum

    def norm: Double = squareRoot(normSquared)

    def fold(other: VectorD)(f: (Double, Double) => Double): VectorD =
      rows.map(index => f(vectorD(index), other(index)))

    def +(other: VectorD): VectorD = fold(other)(_ + _)

    def -(other: VectorD): VectorD = fold(other)(_ - _)

    def *(scalar: Double): VectorD = vectorD.map(_ * scalar)

    def /(scalar: Double): VectorD = vectorD.map(_ / scalar)

    def versor: VectorD = this / norm

    def distance(other: VectorD): Double = (this - other).norm

    def dotProduct(other: VectorD): Double = rows.map(i => vectorD(i) * other(i)).sum

    def tensorProduct(other: VectorD): MatrixD = vectorD.map(other * _)

    def negate: VectorD = vectorD.map(-_)

    def fuzz(other: VectorD)(implicit random: Random): VectorD = fold(other)(_ + _ * random.nextGaussian())
  }

}
