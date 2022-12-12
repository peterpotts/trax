package com.peterpotts.trax.models.f

import com.peterpotts.trax.kalman._
import com.peterpotts.trax.math._

/**
  * @author Peter Potts
  */
case class UpdaterF(north: Vector2) extends Updater {
  val v: Z = ZF(
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
      h = ZF(
        dis = x.d,
        vel = x.v,
        accB = R * x.a,
        magB = R * north + x.magOffB,
        opt = x.omega,
        gyr = x.alpha + x.gyrOff
      ),
      H = ZF.columns(
        dis = XF.twoRows(d = Matrix22.identity),
        vel = XF.twoRows(v = Matrix22.identity),
        accB = XF.twoRows(a = R, theta = dR * x.a),
        magB = XF.twoRows(theta = dR * north, magOffB = Matrix22.identity),
        opt = XF.oneRow(omega = Vector1.identity),
        gyr = XF.oneRow(alpha = Vector1.identity, gyrOff = Vector1.identity)
      )
    )
  }
}
