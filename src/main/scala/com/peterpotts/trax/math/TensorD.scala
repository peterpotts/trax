package com.peterpotts.trax.math

/**
  * @author Peter Potts
  */
object TensorD {
  def constant(I: Int, J: Int, K: Int, x: Double): TensorD = Vector.fill(I)(MatrixD.constant(J, K, x))

  def zero(I: Int, J: Int, K: Int): TensorD = constant(I, J, K, 0)

  def apply(values: MatrixD*): TensorD = values.toVector
}
