package com.peterpotts.trax.math

import com.peterpotts.example.RandomExample._
import com.peterpotts.trax.math.GeometricEquality._
import com.peterpotts.trax.math.GeometryExamples._
import com.peterpotts.trax.math.MatrixDDecorator._
import com.peterpotts.trax.math.VectorDDecorator._
import org.scalatest.matchers.should.Matchers
import org.scalatest.wordspec.AnyWordSpec

/**
  * @author Peter Potts
  */

class Matrix44Test extends AnyWordSpec with Matchers {
  "Matrix 4 by 4" should {
    "evaluate add" in {
      val a = exampleMatrix44.next()
      val b = exampleMatrix44.next()
      val c = (a.matrixD + b.matrixD).matrix44
      a + b shouldEqual c
    }

    "evaluate subtract" in {
      val a = exampleMatrix44.next()
      val b = exampleMatrix44.next()
      val c = (a.matrixD - b.matrixD).matrix44
      a - b shouldEqual c
    }

    "evaluate multiply scalar" in {
      val a = exampleMatrix44.next()
      val b = exampleDouble.next()
      val c = (a.matrixD * b).matrix44
      a * b shouldEqual c
    }

    "evaluate multiply vector" in {
      val a = exampleMatrix44.next()
      val b = exampleVector4.next()
      val c = (a.matrixD ** b.vectorD).vector4
      a * b shouldEqual c
    }

    "evaluate multiply 4 by 3 matrix" in {
      val a = exampleMatrix44.next()
      val b = exampleMatrix43.next()
      val c = (a.matrixD *** b.matrixD).matrix43
      a * b shouldEqual c
    }

    "evaluate multiply 4 by 4 matrix" in {
      val a = exampleMatrix44.next()
      val b = exampleMatrix44.next()
      val c = (a.matrixD *** b.matrixD).matrix44
      a * b shouldEqual c
    }

    "evaluate divide scalar" in {
      val a = exampleMatrix44.next()
      val b = exampleDouble.next()
      val c = (a.matrixD / b).matrix44
      a / b shouldEqual c
    }

    "evaluate transpose" in {
      val a = exampleMatrix44.next()
      val b = a.matrixD.transpose.matrix44
      a.transpose shouldEqual b
    }

    "satisfy identity property for multiplication property" in {
      val a = exampleMatrix44.next()
      a * Matrix44.identity shouldEqual a
    }

    "satisfy transpose identity to identity property" in {
      Matrix44.identity.transpose shouldEqual Matrix44.identity
    }

    "satisfy associative property for multiplication" in {
      val a = exampleMatrix44.next()
      val b = exampleMatrix44.next()
      val c = exampleMatrix44.next()
      (a * b) * c shouldEqual (a * (b * c))
    }

    "satisfy transitive property for multiplication and addition" in {
      val a = exampleMatrix44.next()
      val b = exampleMatrix44.next()
      val c = exampleMatrix44.next()
      (a + b) * c shouldEqual (a * c) + (b * c)
    }

    "satisfy transpose property for multiplication" in {
      val a = exampleMatrix44.next()
      val b = exampleMatrix44.next()
      a.transpose * b.transpose shouldEqual (b * a).transpose
    }
  }
}
