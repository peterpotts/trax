package com.peterpotts.trax.models

import com.peterpotts.trax.kalman._

/**
  * @author Peter Potts
  */
case class Step(time: Double, actual: X, target: X, estimate: Estimate, z: Z, u: U)
