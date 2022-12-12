package com.peterpotts.trax.kalman

import com.peterpotts.trax.math.VectorDDecorator._
import com.peterpotts.trax.math._

import scala.util.Random

/**
  * @author Peter Potts
  */
trait Predictor {
  implicit val random: Random = new Random(0L)

  /**
    * Standard deviation of input.
    */
  val w: U

  /**
    * Input covariance matrix.
    */
  lazy val Q: MatrixD = MatrixD.diagonal(w.vectorD.squared)

  def fuzzPredict(x: X, u: U): Prediction = predict(x, fuzz(u))

  def fuzz(u: U): U = u fuzz w

  def predict(x: X, u: U): Prediction
}
