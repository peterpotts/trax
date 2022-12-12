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
class Matrix43Test extends AnyWordSpec with Matchers {
  "Matrix 4 by 3" should {
    "evaluate add" in {
      val a = exampleMatrix43.next()
      val b = exampleMatrix43.next()
      val c = (a.matrixD + b.matrixD).matrix43
      a + b shouldEqual c
    }

    "evaluate subtract" in {
      val a = exampleMatrix43.next()
      val b = exampleMatrix43.next()
      val c = (a.matrixD - b.matrixD).matrix43
      a - b shouldEqual c
    }

    "evaluate multiply scalar" in {
      val a = exampleMatrix43.next()
      val b = exampleDouble.next()
      val c = (a.matrixD * b).matrix43
      a * b shouldEqual c
    }

    "evaluate multiply vector" in {
      val a = exampleMatrix43.next()
      val b = exampleVector3.next()
      val c = (a.matrixD ** b.vectorD).vector4
      a * b shouldEqual c
    }

    "evaluate multiply 3 by 3 matrix" in {
      val a = exampleMatrix43.next()
      val b = exampleMatrix33.next()
      val c = (a.matrixD *** b.matrixD).matrix43
      a * b shouldEqual c
    }

    "evaluate multiply 3 by 4 matrix" in {
      val a = exampleMatrix43.next()
      val b = exampleMatrix34.next()
      val c = (a.matrixD *** b.matrixD).matrix44
      a * b shouldEqual c
    }

    "evaluate divide scalar" in {
      val a = exampleMatrix43.next()
      val b = exampleDouble.next()
      val c = (a.matrixD / b).matrix43
      a / b shouldEqual c
    }

    "evaluate transpose" in {
      val a = exampleMatrix43.next()
      val b = a.matrixD.transpose.matrix34
      a.transpose shouldEqual b
    }

    "satisfy transpose property for multiplication" in {
      val a = exampleMatrix43.next()
      val b = exampleMatrix34.next()
      a.transpose * b.transpose shouldEqual (b * a).transpose
    }
  }
}
