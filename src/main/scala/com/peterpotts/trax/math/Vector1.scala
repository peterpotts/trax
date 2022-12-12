package com.peterpotts.trax.math

import com.peterpotts.trax.math.MathDecorator._

import scala.math._

/**
  * @author Peter Potts
  */
case class Vector1(value: Double) extends VectorLike {
  require(!value.isNaN, "value must be a number")
  lazy val vectorD: VectorD = Vector(value)
  lazy val norm: Double = abs(value)
  lazy val versor: Vector1 = Vector1(1)

  def ===(that: Vector1): Boolean = value === that.value

  def fold(that: Vector1)(f: (Double, Double) => Double): Vector1 = Vector1(f(value, that.value))

  def map1(f: Double => Double): Vector1 = Vector1(f(value))

  def distance(that: Vector1): Double = (this - that).norm

  def +(that: Vector1): Vector1 = fold(that)(_ + _)

  def -(that: Vector1): Vector1 = fold(that)(_ - _)

  def *(scalar: Double): Vector1 = map1(_ * scalar)

  def /(scalar: Double): Vector1 = map1(_ / scalar)

  def dotProduct(that: Vector1): Double = value * that.value

  lazy val negate: Vector1 = Vector1(-value)

  lazy val inverse: Vector1 = Vector1(1 / value)
}

object Vector1 {
  val zero: Vector1 = Vector1(0)
  val identity: Vector1 = Vector1(1)

  def constant(value: Double): Vector1 = Vector1(value)
}
