package com.peterpotts.trax

import scala.math._

/**
  * @author Peter Potts
  */
package object math {
  val epsilon: Double = 1e-6

  type VectorD = Vector[Double]
  type MatrixD = Vector[VectorD]
  type TensorD = Vector[MatrixD]

  val π: Double = Pi

  val HalfPi: Double = Pi / 2
  val OneHalfPi: Double = Pi * 3 / 2
  val TwoPi: Double = 2 * Pi
  val ThreePi: Double = 3 * Pi
  val FourPi: Double = 4 * Pi

  val MinusHalfPi: Double = -HalfPi
  val MinusPi: Double = -Pi
  val MinusOneHalfPi: Double = -OneHalfPi
  val MinusTwoPi: Double = -TwoPi
  val MinusThreePi: Double = -ThreePi
  val MinusFourPi: Double = -FourPi

  def squareRoot(x: Double): Double = if (x < 0) 0 else sqrt(x)

  def arcSin(x: Double): Double = if (x <= -1) MinusHalfPi else if (x >= 1) HalfPi else asin(x)

  def arcCos(x: Double): Double = if (x <= -1) Pi else if (x >= 1) 0 else acos(x)

  def arcTan2(y: Double, x: Double): Double = if (y == 0 && x < 0) Pi else atan2(y, x)
}
