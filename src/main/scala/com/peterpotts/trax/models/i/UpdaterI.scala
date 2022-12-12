package com.peterpotts.trax.models.i

import com.peterpotts.trax.kalman._
import com.peterpotts.trax.math._

/**
  * @author Peter Potts
  */
case class UpdaterI(north: Vector3) extends Updater {
  val v: Z = ZI(
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
      h = ZI(
        dis = x.d,
        vel = x.v,
        accB = R * x.a,
        magB = R * north,
        optB = x.omegaB,
        gyrB = x.alphaB
      ),
      H = ZI.columns(
        dis = XI.row(d = Matrix33.identity),
        vel = XI.row(v = Matrix33.identity),
        accB = XI.row(
          a = R,
          r = x.r.jacobianRotationMatrix.ikj * x.a
        ),
        magB = XI.row(r = x.r.jacobianRotationMatrix.ikj * north),
        optB = XI.row(omegaB = Matrix33.identity),
        gyrB = XI.row(alphaB = Matrix33.identity)
      )
    )
  }
}
