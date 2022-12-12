package com.peterpotts.trax.math

import com.peterpotts.trax.math.DoubleEquality._
import org.scalatest.matchers.should.Matchers
import org.scalatest.wordspec.AnyWordSpec

/**
  * @author Peter Potts
  */
class TetrahedralTest extends AnyWordSpec with Matchers {
  "Tetrahedral" should {
    "limit" in {
      import Tetrahedral._
      w.norm shouldEqual 1.0
      x.norm shouldEqual 1.0
      y.norm shouldEqual 1.0
      z.norm shouldEqual 1.0
      w dotProduct x shouldEqual limit
      w dotProduct y shouldEqual limit
      w dotProduct x shouldEqual limit
      x dotProduct y shouldEqual limit
      x dotProduct z shouldEqual limit
      y dotProduct z shouldEqual limit
    }
  }
}
