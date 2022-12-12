package com.peterpotts.trax.math

import com.peterpotts.trax.math.MathDecorator._

/**
  * @author Peter Potts
  */
case class DiagonalMatrix33(vector3: Vector3) {
  lazy val matrix33: Matrix33 = Matrix33.diagonal(vector3)
  lazy val inverse: DiagonalMatrix33 = DiagonalMatrix33(vector3.map1(_.reciprocal))
  lazy val squared: DiagonalMatrix33 = DiagonalMatrix33(vector3.map1(_.squared))

  def *(scalar: Double): Matrix33 = Matrix33.diagonal(vector3 * scalar)

  def *(other: Vector3): Vector3 = vector3.fold(other)(_ * _)

  def *(other: Matrix33): Matrix33 = vector3.fold2(other)((scalar, vector) => vector * scalar)

  def /(scalar: Double): Matrix33 = Matrix33.diagonal(vector3 / scalar)
}
