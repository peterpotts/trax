package com.peterpotts.trax.models.d

import com.peterpotts.trax.kalman._
import com.peterpotts.trax.math._

/**
  * @author Peter Potts
  */
case class UpdaterD(north: Vector2) extends Updater {
  val v: Z = ZD(
    dis = Vector2.constant(1.0),
    vel = Vector2.constant(0.06),
    accB = Vector2.constant(0.004),
    magB = Vector2.constant(0.2),
    gyr = Vector1.constant(0.02)
  )

  def update(x: X): Update = {
    val R = x.theta.rotationMatrix.matrix22
    val dR = x.theta.jacobianRotationMatrix

    Update(
      h = ZD(
        dis = x.d,
        vel = x.v,
        accB = R * x.a,
        magB = R * north,
        gyr = x.alpha
      ),
      H = ZD.columns(
        dis = XD.twoRows(d = Matrix22.identity),
        vel = XD.twoRows(v = Matrix22.identity),
        accB = XD.twoRows(a = R, theta = dR * x.a),
        magB = XD.twoRows(theta = dR * north),
        gyr = XD.oneRow(alpha = Vector1.identity)
      )
    )
  }
}
