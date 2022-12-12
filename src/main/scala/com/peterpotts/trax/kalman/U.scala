package com.peterpotts.trax.kalman

import com.peterpotts.trax.math.VectorDDecorator._
import com.peterpotts.trax.math._

import scala.util.Random

/**
  * Input vector.
  *
  * @author Peter Potts
  */
case class U(vectorD: VectorD) {
  def is(I: Int): Boolean = vectorD.is(I)

  def apply(i: Int): Double = vectorD(i)

  def fuzz(w: U)(implicit random: Random): U = U(vectorD fuzz w.vectorD)

  def ===(that: U): Boolean = vectorD === that.vectorD

  override def toString: String = vectorD.map(_ formatted "%.3f").mkString("U(", ", ", ")")
}
