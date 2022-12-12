package com.peterpotts.trax.kalman

import com.peterpotts.trax.math.MatrixD
import com.peterpotts.trax.math.MatrixDDecorator._
import com.peterpotts.trax.math.VectorDDecorator._

/**
  * @author Peter Potts
  */
case class ExtendedKalmanFilter(
  nx: Int,
  nu: Int,
  nz: Int,
  predictor: Predictor,
  updater: Updater
) extends Filter {
  require(predictor.Q.is(nu, nu), s"Q must be $nu by $nu matrix")
  require(updater.R.is(nz, nz), s"R must be $nz by $nz matrix")
  private val I = MatrixD.identity(nx)

  def predict(u: U)(estimate: Estimate): Estimate = {
    require(u.is(nu), s"u must be $nu vector")
    require(estimate.x.is(nx), s"x must be $nx vector")
    require(estimate.P.is(nx, nx), s"P must be $nx by $nx matrix")
    val prediction = predictor.fuzzPredict(estimate.x, u)
    import prediction._
    require(f.is(nx), s"f must be $nx vector")
    require(F.is(nx, nx), s"F must be $nx by $nx matrix")
    require(G.is(nx, nu), s"G must be $nx by $nu matrix")
    val P = F *** estimate.P *** F.transpose + G *** predictor.Q *** G.transpose
    require(P.is(nx, nx), s"P must be $nx by $nx matrix")
    Estimate(f, P)
  }

  def update(z: Z)(estimate: Estimate): Estimate = {
    require(z.is(nz), s"z must be $nz vector")
    require(estimate.x.is(nx), s"x must be $nx vector")
    require(estimate.P.is(nx, nx), s"P must be $nx by $nx matrix")
    val update = updater.update(estimate.x)
    import update._
    require(h.is(nz), s"h must be $nz vector")
    require(H.is(nz, nx), s"H must be $nz by $nx matrix")
    val covariance = H *** estimate.P *** H.transpose + updater.R
    require(covariance.is(nz, nz), s"covariance must be $nz by $nz matrix")
    val K = estimate.P *** H.transpose *** covariance.inverse
    require(K.is(nx, nz), s"K must be $nx by $nz matrix")
    val x = X(estimate.x.vectorD + K ** (z.vectorD - h.vectorD))
    require(x.is(nx), s"x must be $nx vector")
    val P = (I - K *** H) *** estimate.P
    require(P.is(nx, nx), s"P must be $nx by $nx matrix")
    Estimate(x, P)
  }
}
