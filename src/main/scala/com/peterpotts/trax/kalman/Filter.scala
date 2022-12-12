package com.peterpotts.trax.kalman

/**
  * @author Peter Potts
  */
trait Filter {
  def predict(u: U)(estimate: Estimate): Estimate

  def update(z: Z)(estimate: Estimate): Estimate
}
