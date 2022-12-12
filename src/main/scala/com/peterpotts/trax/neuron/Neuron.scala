package com.peterpotts.trax.neuron

/**
  * @author Peter Potts
  */
case class Neuron(weights: Vector[Double]) {
  def output(inputs: Vector[Double]): Double =
    Sigmoid(weights.zip(inputs).map {
      case (weight, input) => weight * input
    }.sum)
}
