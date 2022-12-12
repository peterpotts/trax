package com.peterpotts.trax.models.g

import com.peterpotts.trax.kalman._
import com.peterpotts.trax.math._

/**
  * @author Peter Potts
  */
case class UpdaterG(north: Vector2) extends Updater {
  val v: Z = ZG(
    dis = Vector2.constant(1.0),
    vel = Vector2.constant(0.06),
    accB = Vector2.constant(0.004),
    magB = Vector2.constant(0.1),
    opt = Vector1.constant(0.006),
    gyr = Vector1.constant(0.01)
  )

  def update(x: X): Update = {
    val R = x.theta.rotationMatrix.matrix22
    val dR = x.theta.jacobianRotationMatrix

    Update(
      h = ZG(
        dis = x.d,
        vel = x.v,
        accB = R * x.a,
        magB = R * north,
        opt = x.omega,
        gyr = x.alpha
      ),
      H = ZG.columns(
        dis = XG.twoRows(d = Matrix22.identity),
        vel = XG.twoRows(v = Matrix22.identity),
        accB = XG.twoRows(a = R, theta = dR * x.a),
        magB = XG.twoRows(theta = dR * north),
        opt = XG.oneRow(omega = Vector1.identity),
        gyr = XG.oneRow(alpha = Vector1.identity)
      )
    )
  }
}
