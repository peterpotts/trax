package com.peterpotts.trax.math

/**
  * @author Peter Potts
  */
trait TensorLike {
  val tensorD: TensorD
  val transposeIJ: TensorLike
  val transposeJK: TensorLike
  val transposeIK: TensorLike
}

