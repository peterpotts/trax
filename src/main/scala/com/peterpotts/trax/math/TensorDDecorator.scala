package com.peterpotts.trax.math

import com.peterpotts.trax.math.MatrixDDecorator._

/**
  * @author Peter Potts
  */
object TensorDDecorator {

  implicit class DecoratedTensorD(val tensorD: TensorD) extends AnyVal {
    def is(rows: Int, columns: Int, layers: Int): Boolean =
      tensorD.size == rows &&
        tensorD.forall(_.size == columns) &&
        tensorD.forall(matrix => matrix.forall(_.size == layers))

    def rows: Vector[Int] = tensorD.indices.toVector

    def columns: Vector[Int] = tensorD.head.indices.toVector

    def layers: Vector[Int] = tensorD.head.head.indices.toVector

    def tensor333: Tensor333 = {
      require(tensorD.is(3, 3, 3), "tensorD must be 3 by 3 by 3 tensor")
      Tensor333(tensorD(0).matrix33, tensorD(1).matrix33, tensorD(2).matrix33)
    }

    def tensor334: Tensor334 = {
      require(tensorD.is(3, 3, 4), "tensorD must be 3 by 3 by 4 tensor")
      Tensor334(tensorD(0).matrix34, tensorD(1).matrix34, tensorD(2).matrix34)
    }

    def tensor343: Tensor343 = {
      require(tensorD.is(3, 4, 3), "tensorD must be 3 by 4 by 3 tensor")
      Tensor343(tensorD(0).matrix43, tensorD(1).matrix43, tensorD(2).matrix43)
    }

    def tensor433: Tensor433 = {
      require(tensorD.is(4, 3, 3), "tensorD must be 4 by 3 by 3 tensor")
      Tensor433(tensorD(0).matrix33, tensorD(1).matrix33, tensorD(2).matrix33, tensorD(3).matrix33)
    }

    def fold(other: TensorD)(f: (MatrixD, MatrixD) => MatrixD): TensorD =
      rows.map(index => f(tensorD(index), other(index)))

    def +(other: TensorD): TensorD = fold(other)(_ + _)

    def -(other: TensorD): TensorD = fold(other)(_ - _)

    def *(scalar: Double): TensorD = tensorD.map(_ * scalar)

    def **(vector: VectorD): MatrixD = tensorD.map(_ ** vector)

    def ***(matrix: MatrixD): TensorD = tensorD.map(_ *** matrix)

    def /(scalar: Double): TensorD = tensorD.map(_ / scalar)

    def transposeIJ: TensorD = tensorD.transpose

    def transposeJK: TensorD = tensorD.map(_.transpose)

    def transposeIK: TensorD = transposeIJ.transposeJK.transposeIJ
  }

}
