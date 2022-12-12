package com.peterpotts.trax.math

/**
  * @author Peter Potts
  */
trait VectorLike extends MatrixLike {
  val vectorD: VectorD
  lazy val matrixD: MatrixD = vectorD.map(VectorD(_))
}
