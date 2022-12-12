package com.peterpotts.trax.math

import com.peterpotts.trax.math.MathDecorator._

/**
  * Passive transformation rotation matrix. The active transformation rotation matrix is just the transpose. An active
  * transformation is used to change a position while a passive transformation is used to change a coordinate system.
  *
  * @author Peter Potts
  */
case class RotationMatrix22(matrix22: Matrix22) {
  require(matrix22.determinant === 1, "matrix must have unit determinant")
  require((matrix22 * matrix22.transpose) === Matrix22.identity, "matrix transpose must be matrix inverse")

  def *(other: RotationMatrix22): RotationMatrix22 = RotationMatrix22(matrix22 * other.matrix22)

  lazy val inverse: RotationMatrix22 = RotationMatrix22(matrix22.transpose)
}
