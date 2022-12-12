package com.peterpotts.trax.math

import scala.math._

/**
  * @author Peter Potts
  */
object Tetrahedral {
  val limit: Double = -1.0 / 3.0
  private val theta = arcCos(limit)
  val w = Vector3(1, 0, 0)
  val x = Vector3(cos(theta), sin(theta), 0)
  val y = Vector3(cos(theta), sin(theta) * cos(2 * Pi / 3), sin(theta) * sin(2 * Pi / 3))
  val z = Vector3(cos(theta), sin(theta) * cos(4 * Pi / 3), sin(theta) * sin(4 * Pi / 3))
}
