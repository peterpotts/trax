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
class Matrix34Test extends AnyWordSpec with Matchers {
  "Matrix 3 by 4" should {
    "evaluate add" in {
      val a = exampleMatrix34.next()
      val b = exampleMatrix34.next()
      val c = (a.matrixD + b.matrixD).matrix34
      a + b shouldEqual c
    }

    "evaluate subtract" in {
      val a = exampleMatrix34.next()
      val b = exampleMatrix34.next()
      val c = (a.matrixD - b.matrixD).matrix34
      a - b shouldEqual c
    }

    "evaluate multiply scalar" in {
      val a = exampleMatrix34.next()
      val b = exampleDouble.next()
      val c = (a.matrixD * b).matrix34
      a * b shouldEqual c
    }

    "evaluate multiply vector" in {
      val a = exampleMatrix34.next()
      val b = exampleVector4.next()
      val c = (a.matrixD ** b.vectorD).vector3
      a * b shouldEqual c
    }

    "evaluate multiply 4 by 3 matrix" in {
      val a = exampleMatrix34.next()
      val b = exampleMatrix43.next()
      val c = (a.matrixD *** b.matrixD).matrix33
      a * b shouldEqual c
    }

    "evaluate multiply 4 by 4 matrix" in {
      val a = exampleMatrix34.next()
      val b = exampleMatrix44.next()
      val c = (a.matrixD *** b.matrixD).matrix34
      a * b shouldEqual c
    }

    "evaluate divide scalar" in {
      val a = exampleMatrix34.next()
      val b = exampleDouble.next()
      val c = (a.matrixD / b).matrix34
      a / b shouldEqual c
    }

    "evaluate transpose" in {
      val a = exampleMatrix34.next()
      val b = a.matrixD.transpose.matrix43
      a.transpose shouldEqual b
    }

    "satisfy transpose property for multiplication" in {
      val a = exampleMatrix34.next()
      val b = exampleMatrix43.next()
      a.transpose * b.transpose shouldEqual (b * a).transpose
    }
  }
}
