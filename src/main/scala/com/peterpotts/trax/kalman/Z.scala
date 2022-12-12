package com.peterpotts.trax.kalman

import com.peterpotts.trax.math.VectorDDecorator._
import com.peterpotts.trax.math._

/**
  * Measurement vector.
  *
  * @author Peter Potts
  */
case class Z(vectorD: VectorD) {
  def is(rows: Int): Boolean = vectorD.is(rows)

  def apply(row: Int): Double = vectorD(row)

  def ===(that: X): Boolean = vectorD === that.vectorD

  override def toString: String = vectorD.map(_ formatted "%.3f").mkString("Z(", ", ", ")")
}
