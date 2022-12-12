package com.peterpotts.trax.models.k

import com.peterpotts.trax.kalman._
import com.peterpotts.trax.math._

case object UpdaterK extends Updater {
  val v: Z = ZK(
    dis = Vector3.constant(1.0),
    vel = Vector3.constant(0.06),
    accB = Vector3.constant(0.004),
    magB = Vector3.constant(0.1),
    optB = Vector3.constant(0.006),
    gyrB = Vector3.constant(0.01)
  )

  def update(x: X): Update = {
    val R = x.e.rotationMatrix.matrix33

    Update(
      h = ZK(
        dis = x.d,
        vel = x.v,
        accB = R * x.a,
        magB = R * x.N,
        optB = x.omegaB,
        gyrB = x.alphaB
      ),
      H = ZK.columns(
        dis = XK.row(d = Matrix33.identity),
        vel = XK.row(v = Matrix33.identity),
        accB = XK.row(
          a = R,
          e = x.e.jacobianRotationMatrix.ikj * x.a
        ),
        magB = XK.row(
          e = x.e.jacobianRotationMatrix.ikj * x.N
        ),
        optB = XK.row(omegaB = Matrix33.identity),
        gyrB = XK.row(alphaB = Matrix33.identity)
      )
    )
  }
}
