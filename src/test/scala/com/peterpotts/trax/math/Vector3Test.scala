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
class Vector3Test extends AnyWordSpec with Matchers {
  "Vector 3" should {
    "evaluate norm" in {
      val a = exampleVector3.next()
      val b = a.vectorD.norm
      a.norm shouldEqual b
    }

    "evaluate add" in {
      val a = exampleVector3.next()
      val b = exampleVector3.next()
      val c = (a.vectorD + b.vectorD).vector3
      a + b shouldEqual c
    }

    "evaluate subtract" in {
      val a = exampleVector3.next()
      val b = exampleVector3.next()
      val c = (a.vectorD - b.vectorD).vector3
      a - b shouldEqual c
    }

    "evaluate multiply" in {
      val a = exampleVector3.next()
      val b = exampleDouble.next()
      val c = (a.vectorD * b).vector3
      a * b shouldEqual c
    }

    "evaluate divide" in {
      val a = exampleVector3.next()
      val b = exampleDouble.next()
      val c = (a.vectorD / b).vector3
      a / b shouldEqual c
    }

    "evaluate distance" in {
      val a = exampleVector3.next()
      val b = exampleVector3.next()
      val c = a.vectorD distance b.vectorD
      a distance b shouldEqual c
    }

    "evaluate dot product" in {
      val a = exampleVector3.next()
      val b = exampleVector3.next()
      val c = a.vectorD dotProduct b.vectorD
      a dotProduct b shouldEqual c
    }

    "evaluate tensor product" in {
      Vector3(1, 2, 3) tensorProduct Vector3(4, 5, 6) shouldEqual
        Matrix33(Vector3(4, 5, 6), Vector3(8, 10, 12), Vector3(12, 15, 18))
    }

    "evaluate cross product" in {
      Vector3(1, 2, 3) crossProduct Vector3(4, 5, 6) shouldEqual Vector3(-3, 6, -3)
    }

    "evaluate negate" in {
      val a = exampleVector3.next()
      val b = a.vectorD.negate.vector3
      a.negate shouldEqual b
    }

    "satisfy cross and dot to zero property" in {
      val a = exampleVector3.next()
      val b = exampleVector3.next()
      (a crossProduct b) dotProduct a shouldEqual 0.0
    }

    "satisfy negation property" in {
      val a = exampleVector3.next()
      a + a.negate shouldEqual Vector3.zero
    }

    "satisfy norm property" in {
      val a = exampleVector3.next()
      (a / a.norm).norm shouldEqual 1.0
    }

    "satisfy skew symmetric property" in {
      val a = exampleVector3.next()
      a.skewSymmetric * a shouldEqual Vector3.zero
    }

    "satisfy cross product to skew symmetric property" in {
      val a = exampleVector3.next()
      val b = exampleVector3.next()
      a crossProduct b shouldEqual a.skewSymmetric * b
    }
  }
}
