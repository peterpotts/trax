package com.peterpotts.trax.models.h

import com.peterpotts.trax.kalman._
import com.peterpotts.trax.math._

/**
  * @author Peter Potts
  */
case class UpdaterH(north: Vector3) extends Updater {
  val v: Z = ZH(
    dis = Vector3.constant(1.0),
    vel = Vector3.constant(0.06),
    accB = Vector3.constant(0.004),
    magB = Vector3.constant(0.1),
    optB = Vector3.constant(0.006),
    gyrB = Vector3.constant(0.01)
  )

  def update(x: X): Update = {
    val R = x.q.rotationMatrix.matrix33

    Update(
      h = ZH(
        dis = x.d,
        vel = x.v,
        accB = R * x.a,
        magB = R * north,
        optB = x.omegaB,
        gyrB = x.alphaB
      ),
      H = ZH.columns(
        dis = XH.threeRows(d = Matrix33.identity),
        vel = XH.threeRows(v = Matrix33.identity),
        accB = XH.threeRows(
          a = R,
          q = x.q.jacobianRotationMatrix.ikj * x.a
        ),
        magB = XH.threeRows(q = x.q.jacobianRotationMatrix.ikj * north),
        optB = XH.threeRows(omegaB = Matrix33.identity),
        gyrB = XH.threeRows(alphaB = Matrix33.identity)
      )
    )
  }
}
