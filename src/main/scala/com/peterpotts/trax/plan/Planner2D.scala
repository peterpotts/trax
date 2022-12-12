package com.peterpotts.trax.plan

import com.peterpotts.trax.kalman.X

/**
  * @author Peter Potts
  */
trait Planner2D {
  val initial: Body2D

  def plan(time: Double, x: X, dt: Double): Body2D
}
