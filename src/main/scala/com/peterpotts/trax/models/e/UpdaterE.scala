package com.peterpotts.trax.models.e

import com.peterpotts.trax.kalman._
import com.peterpotts.trax.math._

/**
  * @author Peter Potts
  */
case class UpdaterE(north: Vector2) extends Updater {
  val v: Z = ZE(
    dis = Vector2.constant(1.0),
    vel = Vector2.constant(0.06),
    accB = Vector2.constant(0.004),
    magB = Vector2.constant(0.2),
    opt = Vector1.constant(0.006),
    gyr = Vector1.constant(0.02)
  )

  def update(x: X): Update = {
    val R = x.theta.rotationMatrix.matrix22
    val dR = x.theta.jacobianRotationMatrix

    Update(
      h = ZE(
        dis = x.d,
        vel = x.v,
        accB = R * x.a,
        magB = R * north + x.magOffB,
        opt = x.omega,
        gyr = x.alpha + x.gyrOff
      ),
      H = ZE.columns(
        dis = XE.twoRows(d = Matrix22.identity),
        vel = XE.twoRows(v = Matrix22.identity),
        accB = XE.twoRows(a = R, theta = dR * x.a),
        magB = XE.twoRows(theta = dR * north),
        opt = XE.oneRow(omega = Vector1.identity),
        gyr = XE.oneRow(alpha = Vector1.identity, gyrOff = Vector1.identity)
      )
    )
  }
}
