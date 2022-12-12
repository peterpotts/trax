package com.peterpotts.trax.math

/**
  * @author Peter Potts
  */
object VectorD {
  def constant(I: Int, x: Double): VectorD = Vector.fill(I)(x)

  def zero(I: Int): VectorD = constant(I, 0)

  def spike(I: Int, i: Int, x: Double): VectorD = Vector.tabulate(I)(j => if (j == i) x else 0)

  def unit(I: Int, i: Int): VectorD = spike(I, i, 1)

  def apply(values: Double*): VectorD = values.toVector

  def vertical(vectors: VectorLike*): VectorD = vectors.toVector.flatMap(_.vectorD)
}
