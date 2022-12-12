package com.peterpotts.trax.kalman

import scala.util.Random

/**
  * @author Peter Potts
  */
case class Moment(t: X, u: U, z: Z) {
  def fuzz(w: U, v: Z)(implicit random: Random): Moment = Moment(t, u fuzz w, z fuzz v)
}
