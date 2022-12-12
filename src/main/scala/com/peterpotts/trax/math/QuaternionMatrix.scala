package com.peterpotts.trax.math

/**
  * @author Peter Potts
  */
case class QuaternionMatrix(matrix44: Matrix44) {
  lazy val transpose: QuaternionMatrix = QuaternionMatrix(matrix44.transpose)

  def *(quaternion: Quaternion): Quaternion = Quaternion(matrix44 * quaternion.vector4)

  def *(matrix43: Matrix43): Matrix43 = matrix44 * matrix43
}
