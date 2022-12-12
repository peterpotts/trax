package com.peterpotts.trax.plan

import com.peterpotts.trax.kalman.X

/**
  * @author Peter Potts
  */
trait Planner3D {
  val initial: Body3D

  def plan(time: Double, x: X, dt: Double): Body3D
}
