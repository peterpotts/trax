package com.peterpotts.trax.math

import com.peterpotts.trax.math.MathDecorator._
import scalafx.geometry.Point3D

/**
  * @author Peter Potts
  */
case class Vector3(x: Double, y: Double, z: Double) extends VectorLike {
  require(!x.isNaN, "x must be a number")
  require(!y.isNaN, "y must be a number")
  require(!z.isNaN, "z must be a number")
  lazy val vectorD: VectorD = Vector(x, y, z)
  lazy val norm: Double = squareRoot(x.squared + y.squared + z.squared)
  lazy val versor: Vector3 = this / norm
  lazy val eulerAngles: EulerAngles = EulerAngles(x, y, z)
  lazy val point: Point3D = new Point3D(x, y, z)

  def ===(that: Vector3): Boolean = x === that.x && y === that.y && z === that.z

  def fold(that: Vector3)(f: (Double, Double) => Double): Vector3 = Vector3(f(x, that.x), f(y, that.y), f(z, that.z))

  def fold2(that: Matrix33)(f: (Double, Vector3) => Vector3): Matrix33 =
    Matrix33(f(x, that.x), f(y, that.y), f(z, that.z))

  def map1(f: Double => Double): Vector3 = Vector3(f(x), f(y), f(z))

  def map3(f: Double => Vector3): Matrix33 = Matrix33(f(x), f(y), f(z))

  def distance(that: Vector3): Double = (this - that).norm

  def +(that: Vector3): Vector3 = fold(that)(_ + _)

  def -(that: Vector3): Vector3 = fold(that)(_ - _)

  def *(scalar: Double): Vector3 = map1(_ * scalar)

  def /(scalar: Double): Vector3 = map1(_ / scalar)

  def dotProduct(that: Vector3): Double = x * that.x + y * that.y + z * that.z

  def tensorProduct(that: Vector3): Matrix33 = map3(that * _)

  def crossProduct(that: Vector3): Vector3 = Vector3(
    y * that.z - z * that.y,
    z * that.x - x * that.z,
    x * that.y - y * that.x
  )

  def ×(that: Vector3): Vector3 = crossProduct(that)

  lazy val negate: Vector3 = Vector3(-x, -y, -z)

  /**
    * Page 14 (105)
    * C(x)
    */
  lazy val skewSymmetric: Matrix33 = Matrix33(Vector3(0, -z, y), Vector3(z, 0, -x), Vector3(-y, x, 0))

  def × : Matrix33 = skewSymmetric
}

object Vector3 {
  val zero: Vector3 = constant(0)
  val X: Vector3 = Vector3(1, 0, 0)
  val Y: Vector3 = Vector3(0, 1, 0)
  val Z: Vector3 = Vector3(0, 0, 1)

  def constant(x: Double): Vector3 = Vector3(x, x, x)
}
