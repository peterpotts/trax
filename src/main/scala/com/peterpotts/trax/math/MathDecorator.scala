package com.peterpotts.trax.math

import scala.math.abs

/**
  * @author Peter Potts
  */
object MathDecorator {

  implicit class DecoratedDouble(val x: Double) extends AnyVal {
    def ===(other: Double): Boolean = abs(x - other) < epsilon * (x.abs max other.abs max 1)

    def vector1: Vector1 = Vector1(x)

    def negate: Double = -x

    def reciprocal: Double = 1 / x

    def squared: Double = x * x

    def cubed: Double = squared * x
  }

  implicit class DecoratedVector[D](val vector: Vector[D]) extends AnyVal {
    def is(size: Int): Boolean = vector.size == size

    def x: D = vector(0)

    def y: D = vector(1)

    def z: D = vector(2)
  }

}
