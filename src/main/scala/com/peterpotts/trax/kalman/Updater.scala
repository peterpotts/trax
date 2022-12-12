package com.peterpotts.trax.kalman

import com.peterpotts.trax.math.MatrixD
import com.peterpotts.trax.math.VectorDDecorator._

/**
  * @author Peter Potts
  */
trait Updater {
  /**
    * Standard deviation of measurement.
    */
  val v: Z

  /**
    * Measurement covariance matrix.
    */
  lazy val R: MatrixD = MatrixD.diagonal(v.vectorD.squared)

  def update(x: X): Update
}
