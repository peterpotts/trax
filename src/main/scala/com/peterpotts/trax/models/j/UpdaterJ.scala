package com.peterpotts.trax.models.j

import com.peterpotts.trax.kalman._
import com.peterpotts.trax.math._

/**
  * @author Peter Potts
  */
case object UpdaterJ extends Updater {
  val v: Z = ZJ(
    dis = Vector3.constant(1.0),
    vel = Vector3.constant(0.06),
    accB = Vector3.constant(0.004),
    magB = Vector3.constant(0.1),
    optB = Vector3.constant(0.006),
    gyrB = Vector3.constant(0.01)
  )

  def update(x: X): Update = {
    val R = x.r.rotationMatrix.matrix33

    Update(
      h = ZJ(
        dis = x.d,
        vel = x.v,
        accB = R * x.a,
        magB = R * x.N,
        optB = x.omegaB,
        gyrB = x.alphaB
      ),
      H = ZJ.columns(
        dis = XJ.row(d = Matrix33.identity),
        vel = XJ.row(v = Matrix33.identity),
        accB = XJ.row(
          a = R,
          r = x.r.jacobianRotationMatrix.ikj * x.a
        ),
        magB = XJ.row(r = x.r.jacobianRotationMatrix.ikj * x.N),
        optB = XJ.row(omegaB = Matrix33.identity),
        gyrB = XJ.row(alphaB = Matrix33.identity)
      )
    )
  }
}
