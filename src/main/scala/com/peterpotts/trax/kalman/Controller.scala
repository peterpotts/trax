package com.peterpotts.trax.kalman

import scala.util.Random

/**
  * @author Peter Potts
  */
trait Controller {
  implicit val random: Random = new Random(0L)

  val initial: Target

  def target(time: Double, x: X, dt: Double): Target

  def measurements(x: X, v: Z): Z

  def regulate(x: X, target: Target): U
}
