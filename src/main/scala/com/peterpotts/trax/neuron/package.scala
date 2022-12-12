package com.peterpotts.trax

import scala.math.exp

/**
  * @author Peter Potts
  */
package object neuron {

  /**
    * @author Peter Potts
    */
  val Sigmoid: Double => Double = value => 1.0 / (1.0 + exp(-value))
}
