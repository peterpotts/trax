package com.peterpotts.trax.lqr

import com.peterpotts.trax.math._

/**
  * @author Peter Potts
  */
trait AngularDynamics3D {
  val inertiaB: DiagonalMatrix33
  val dt: Double
}
