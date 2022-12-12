package com.peterpotts.trax.kalman

import com.peterpotts.trax.math.MatrixD
import com.peterpotts.trax.math.MatrixDDecorator._
import com.peterpotts.trax.math.VectorDDecorator._

/**
  * Kalman Filter
  *
  * @param nx State vector size
  * @param nu Input vector size
  * @param nz Measurement vector size
  * @param F State transition matrix
  * @param G Control matrix
  * @param W Unmeasurable process noise
  * @param H Observation matrix
  * @param Q Input covariance matrix
  * @param R Measurement covariance matrix
  *
  * @author Peter Potts
  */
case class KalmanFilter(
  nx: Int,
  nu: Int,
  nz: Int,
  F: MatrixD,
  G: MatrixD,
  W: MatrixD,
  H: MatrixD,
  Q: MatrixD,
  R: MatrixD
) extends Filter {
  require(F.is(nx, nx), s"F must be $nx by $nx matrix")
  require(G.is(nx, nu), s"G must be $nx by $nu matrix")
  require(W.is(nx, nu), s"W must be $nx by $nu matrix")
  require(H.is(nz, nx), s"H must be $nz by $nx matrix")
  require(Q.is(nu, nu), s"Q must be $nu by $nu matrix")
  require(R.is(nz, nz), s"R must be $nz by $nz matrix")

  private val I = MatrixD.identity(nx)

  def predictX(u: U)(x: X): X = {
    require(u.is(nu), s"u must be $nu vector")
    require(x.is(nx), s"x must be $nx vector")
    X(F ** x.vectorD + G ** u.vectorD)
  }

  private def predictP(P: MatrixD): MatrixD = {
    require(P.is(nx, nx), s"P must be $nx by $nx matrix")
    F *** P *** F.transpose + W *** Q *** W.transpose
  }

  def predict(u: U)(estimate: Estimate): Estimate =
    Estimate(predictX(u)(estimate.x), predictP(estimate.P))

  def gain(P: MatrixD): MatrixD = {
    require(P.is(nx, nx), s"P must be $nx by $nx matrix")
    val covariance = H *** P *** H.transpose + R
    P *** H.transpose *** covariance.inverse
  }

  private def updateX(z: Z)(K: MatrixD)(x: X): X = {
    require(z.is(nz), s"z must be $nz vector")
    require(K.is(nx, nz), s"K must be $nx by $nz matrix")
    require(x.is(nx), s"x must be $nx vector")
    X(x.vectorD + K ** (z.vectorD - H ** x.vectorD))
  }

  private def updateP(K: MatrixD)(P: MatrixD): MatrixD = {
    require(K.is(nx, nz), s"K must be $nx by $nz matrix")
    require(P.is(nx, nx), s"P must be $nx by $nx matrix")
    (I - K *** H) *** P
  }

  def update(z: Z)(estimate: Estimate): Estimate = {
    require(z.is(nz), s"z must be $nz vector")
    val K = gain(estimate.P)
    val x = updateX(z)(K)(estimate.x)
    val P = updateP(K)(estimate.P)
    Estimate(x, P)
  }
}
