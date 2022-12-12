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
class Matrix12Test extends AnyWordSpec with Matchers {
  "Matrix 1 by 2" should {
    "evaluate add" in {
      val a = exampleMatrix12.next()
      val b = exampleMatrix12.next()
      val c = (a.matrixD + b.matrixD).matrix12
      a + b shouldEqual c
    }

    "evaluate subtract" in {
      val a = exampleMatrix12.next()
      val b = exampleMatrix12.next()
      val c = (a.matrixD - b.matrixD).matrix12
      a - b shouldEqual c
    }

    "evaluate multiply scalar" in {
      val a = exampleMatrix12.next()
      val b = exampleDouble.next()
      val c = (a.matrixD * b).matrix12
      a * b shouldEqual c
    }

    "evaluate multiply vector" in {
      val a = exampleMatrix12.next()
      val b = exampleVector2.next()
      val c = (a.matrixD ** b.vectorD).vector1
      a * b shouldEqual c
    }

    "evaluate multiply 2 by 2 matrix" in {
      val a = exampleMatrix12.next()
      val b = exampleMatrix22.next()
      val c = (a.matrixD *** b.matrixD).matrix12
      a * b shouldEqual c
    }

    "evaluate divide scalar" in {
      val a = exampleMatrix12.next()
      val b = exampleDouble.next()
      val c = (a.matrixD / b).matrix12
      a / b shouldEqual c
    }

    "evaluate transpose" in {
      val a = exampleMatrix44.next()
      val b = a.matrixD.transpose.matrix44
      a.transpose shouldEqual b
    }
  }
}
