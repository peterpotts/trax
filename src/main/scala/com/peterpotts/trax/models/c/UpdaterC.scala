package com.peterpotts.trax.models.c

import com.peterpotts.trax.kalman._
import com.peterpotts.trax.math._

/**
  * @author Peter Potts
  */
case object UpdaterC extends Updater {
  /**
    * The normal distributed measurement noise with covariance R.
    */
  val v: Z = ZC(dis = Vector2.constant(1.0))

  def update(x: X): Update =
    Update(
      h = ZC(dis = x.d),
      H = ZC.columns(dis = XC.row(d = Matrix22.identity))
    )
}
