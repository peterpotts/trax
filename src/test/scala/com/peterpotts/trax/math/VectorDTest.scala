package com.peterpotts.trax.math

import com.peterpotts.trax.math.DoubleEquality._
import com.peterpotts.trax.math.GeometryExamples._
import com.peterpotts.trax.math.VectorDDecorator._
import org.scalatest.matchers.should.Matchers
import org.scalatest.wordspec.AnyWordSpec

/**
  * @author Peter Potts
  */
class VectorDTest extends AnyWordSpec with Matchers {
  "Vector D" should {
    "evaluate is" in {
      VectorD(3, 7).is(2) shouldEqual true
      VectorD(3, 7).is(3) shouldEqual false
    }

    "evaluate rows" in {
      VectorD(3, 7).rows shouldEqual Vector(0, 1)
    }

    "convert to matrix" in {
      VectorD(3, 7).matrixD shouldEqual MatrixD(VectorD(3), VectorD(7))
    }

    "evaluate norm" in {
      VectorD(3, 4).norm shouldEqual 5.0
    }

    "evaluate add" in {
      VectorD(1, 2) + VectorD(3, 4) shouldEqual VectorD(4, 6)
    }

    "evaluate subtract" in {
      VectorD(3, 4) - VectorD(1, 2) shouldEqual VectorD(2, 2)
    }

    "evaluate multiply" in {
      VectorD(1, 2) * 3.0 shouldEqual VectorD(3, 6)
    }

    "evaluate divide" in {
      VectorD(2, 4) / 2.0 shouldEqual VectorD(1, 2)
    }

    "evaluate distance" in {
      VectorD(7, 3) distance VectorD(11, 0) shouldEqual 5.0
    }

    "evaluate dot product" in {
      VectorD(1, 2) dotProduct VectorD(3, 4) shouldEqual 11.0
    }

    "evaluate tensor product" in {
      VectorD(0, 3) tensorProduct VectorD(5, -3) shouldEqual MatrixD(VectorD(0, 0), VectorD(15, -9))
    }

    "evaluate negate" in {
      VectorD(1, -2).negate shouldEqual VectorD(-1, 2)
    }

    "satisfy negation property" in {
      val a = exampleVectorD(3).next()
      a + a.negate shouldEqual VectorD.zero(3)
    }

    "satisfy norm property" in {
      val a = exampleVectorD(3).next()
      (a / a.norm).norm shouldEqual 1.0
    }
  }
}
