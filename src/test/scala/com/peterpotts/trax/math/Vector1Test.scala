package com.peterpotts.trax.math

import com.peterpotts.example.RandomExample._
import com.peterpotts.trax.math.DoubleEquality._
import com.peterpotts.trax.math.GeometricEquality._
import com.peterpotts.trax.math.GeometryExamples._
import com.peterpotts.trax.math.VectorDDecorator._
import org.scalatest.matchers.should.Matchers
import org.scalatest.wordspec.AnyWordSpec

/**
  * @author Peter Potts
  */
class Vector1Test extends AnyWordSpec with Matchers {
  "Vector 2" should {
    "evaluate norm" in {
      val a = exampleVector1.next()
      val b = a.vectorD.norm
      a.norm shouldEqual b
    }

    "evaluate add" in {
      val a = exampleVector1.next()
      val b = exampleVector1.next()
      val c = (a.vectorD + b.vectorD).vector1
      a + b shouldEqual c
    }

    "evaluate subtract" in {
      val a = exampleVector1.next()
      val b = exampleVector1.next()
      val c = (a.vectorD - b.vectorD).vector1
      a - b shouldEqual c
    }

    "evaluate multiply" in {
      val a = exampleVector1.next()
      val b = exampleDouble.next()
      val c = (a.vectorD * b).vector1
      a * b shouldEqual c
    }

    "evaluate divide" in {
      val a = exampleVector1.next()
      val b = exampleDouble.next()
      val c = (a.vectorD / b).vector1
      a / b shouldEqual c
    }

    "evaluate distance" in {
      val a = exampleVector1.next()
      val b = exampleVector1.next()
      val c = a.vectorD distance b.vectorD
      a distance b shouldEqual c
    }

    "evaluate dot product" in {
      val a = exampleVector1.next()
      val b = exampleVector1.next()
      val c = a.vectorD dotProduct b.vectorD
      a dotProduct b shouldEqual c
    }

    "evaluate negate" in {
      val a = exampleVector1.next()
      val b = a.vectorD.negate.vector1
      a.negate shouldEqual b
    }

    "satisfy negation property" in {
      val a = exampleVector1.next()
      a + a.negate shouldEqual Vector1.zero
    }

    "satisfy norm property" in {
      val a = exampleVector1.next()
      (a / a.norm).norm shouldEqual 1.0
    }
  }
}
