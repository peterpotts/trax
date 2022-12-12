package com.peterpotts.trax.math

import com.peterpotts.trax.math.VectorDDecorator._

/**
  * @author Peter Potts
  */
object MatrixDDecorator {

  implicit class DecoratedMatrixD(val matrixD: MatrixD) extends AnyVal {
    def is(rows: Int, columns: Int): Boolean =
      matrixD.size == rows && matrixD.forall(_.size == columns)

    def rows: Vector[Int] = matrixD.indices.toVector

    def columns: Vector[Int] = matrixD.head.indices.toVector

    def ===(other: MatrixD): Boolean =
      matrixD.size == other.size && rows.forall(index => matrixD(index) === other(index))

    def matrix12: Matrix12 = {
      require(matrixD.is(1, 2), "matrixD must be 1 by 2 matrix")
      Matrix12(matrixD(0).vector2)
    }

    def matrix22: Matrix22 = {
      require(matrixD.is(2, 2), "matrixD must be 2 by 2 matrix")
      Matrix22(matrixD(0).vector2, matrixD(1).vector2)
    }

    def matrix33: Matrix33 = {
      require(matrixD.is(3, 3), "matrixD must be 3 by 3 matrix")
      Matrix33(matrixD(0).vector3, matrixD(1).vector3, matrixD(2).vector3)
    }

    def matrix34: Matrix34 = {
      require(matrixD.is(3, 4), "matrixD must be 3 by 4 matrix")
      Matrix34(matrixD(0).vector4, matrixD(1).vector4, matrixD(2).vector4)
    }

    def matrix43: Matrix43 = {
      require(matrixD.is(4, 3), "matrixD must be 4 by 3 matrix")
      Matrix43(matrixD(0).vector3, matrixD(1).vector3, matrixD(2).vector3, matrixD(3).vector3)
    }

    def matrix44: Matrix44 = {
      require(matrixD.is(4, 4), "matrixD must be 4 by 4 matrix")
      Matrix44(matrixD(0).vector4, matrixD(1).vector4, matrixD(2).vector4, matrixD(3).vector4)
    }

    def trace: VectorD = rows.map(i => matrixD(i)(i))

    def fold(other: MatrixD)(f: (VectorD, VectorD) => VectorD): MatrixD =
      rows.map(index => f(matrixD(index), other(index)))

    def +(other: MatrixD): MatrixD = fold(other)(_ + _)

    def -(other: MatrixD): MatrixD = fold(other)(_ - _)

    def *(scalar: Double): MatrixD = matrixD.map(_ * scalar)

    def **(vector: VectorD): VectorD = matrixD.map(_ dotProduct vector)

    def ***(other: MatrixD): MatrixD = other.transpose.map(matrixD ** _).transpose

    def /(scalar: Double): MatrixD = matrixD.map(_ / scalar)

    def negate: MatrixD = matrixD.map(_.negate)

    def submatrix(i: Int, j: Int): MatrixD =
      matrixD.patch(i, Vector.empty, 1).map(_.patch(j, Vector.empty, 1))

    def minor(i: Int, j: Int): Double = submatrix(i, j).determinant

    def cofactor(i: Int, j: Int): Double =
      if ((i + j) % 2 == 0) minor(i, j) else -minor(i, j)

    def determinant: Double =
      if (is(1, 1))
        matrixD.head.head
      else
        matrixD.indices.map(j => matrixD.head(j) * cofactor(0, j)).sum

    def transpose: MatrixD = matrixD.transpose

    def comatrix: MatrixD =
      rows.map(i => rows.map(j => cofactor(i, j)))

    def adjugate: MatrixD = comatrix.transpose

    def inverse: MatrixD =
      if (is(1, 1))
        matrixD.map(_.map(1.0 / _))
      else
        MatrixInverse(matrixD)

    def slowInverse: MatrixD =
      if (is(1, 1))
        matrixD.map(_.map(1.0 / _))
      else
        adjugate / determinant
  }

}
