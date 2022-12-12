package com.peterpotts.trax

import com.peterpotts.trax.math.VectorDDecorator._

import scala.util.Random

/**
  * @author Peter Potts
  */
package object kalman {
  implicit class DecoratedZ(val z: Z) extends AnyVal {
    def fuzz(v: Z)(implicit random: Random): Z = Z(z.vectorD fuzz v.vectorD)
  }

}
