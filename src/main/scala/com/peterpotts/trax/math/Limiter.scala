package com.peterpotts.trax.math

/**
  * @author Peter Potts
  */
case class Limiter(lowerBound: Double = Double.MinValue, upperBound: Double = Double.MaxValue) {
  def apply(value: Double): Double =
    if (value < lowerBound)
      lowerBound
    else if (value > upperBound)
      upperBound
    else
      value
}
