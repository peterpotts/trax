package com.peterpotts.trax.math

import com.peterpotts.trax.math.VectorDDecorator._

/**
  * @author Peter Potts
  */
object MatrixD {
  def constant(I: Int, J: Int, x: Double): MatrixD = Vector.fill(I)(VectorD.constant(J, x))

  def zero(I: Int, J: Int): MatrixD = constant(I, J, 0)

  def identity(I: Int): MatrixD = Vector.tabulate(I)(i => VectorD.unit(I, i))

  def diagonal(vector: VectorD): MatrixD = vector.rows.map(i => VectorD.spike(vector.size, i, vector(i)))

  def apply(vectors: VectorD*): MatrixD = vectors.toVector

  def horizontal(matrices: MatrixLike*): MatrixD = matrices.head.matrixD.indices.toVector.map { index =>
    matrices.toVector.flatMap(matrix => matrix.matrixD(index))
  }

  def vertical(matrices: MatrixD*): MatrixD = matrices.toVector.flatten
}
