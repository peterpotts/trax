package com.peterpotts.trax.math

import com.peterpotts.example.RandomExample.{exampleDouble, _}
import com.peterpotts.trax.math.DoubleEquality._
import com.peterpotts.trax.math.GeometricEquality._
import com.peterpotts.trax.math.GeometryExamples._
import com.peterpotts.trax.math.VectorDDecorator._
import org.scalatest.matchers.should.Matchers
import org.scalatest.wordspec.AnyWordSpec

/**
  * @author Peter Potts
  */
class Vector4Test extends AnyWordSpec with Matchers {
  "Vector 4" should {
    "evaluate add" in {
      val a = exampleVector4.next()
      val b = exampleVector4.next()
      val c = (a.vectorD + b.vectorD).vector4
      a + b shouldEqual c
    }

    "evaluate subtract" in {
      val a = exampleVector4.next()
      val b = exampleVector4.next()
      val c = (a.vectorD - b.vectorD).vector4
      a - b shouldEqual c
    }

    "evaluate multiply" in {
      val a = exampleVector4.next()
      val b = exampleDouble.next()
      val c = (a.vectorD * b).vector4
      a * b shouldEqual c
    }

    "evaluate divide" in {
      val a = exampleVector4.next()
      val b = exampleDouble.next()
      val c = (a.vectorD / b).vector4
      a / b shouldEqual c
    }

    "evaluate dot product" in {
      val a = exampleVector4.next()
      val b = exampleVector4.next()
      val c = a.vectorD dotProduct b.vectorD
      a dotProduct b shouldEqual c
    }

    "evaluate negate" in {
      val a = exampleVector4.next()
      val b = a.vectorD.negate.vector4
      a.negate shouldEqual b
    }

    "satisfy negation property" in {
      val a = exampleVector4.next()
      a + a.negate shouldEqual Vector4.zero
    }
  }
}
