package com.peterpotts.trax.models

import com.peterpotts.trax.math.EulerAngles

/**
  * @author Peter Potts
  */
trait Model3D extends Model {
  def toEulerAngles(step: Step): EulerAngles
}
