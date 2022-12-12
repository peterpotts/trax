package com.peterpotts.trax.common

/**
  * @author Peter Potts
  */
object LoanPattern {
  def using[R, T](resource: R)(release: R => Unit)(use: R => T): T = try use(resource) finally release(resource)
}

