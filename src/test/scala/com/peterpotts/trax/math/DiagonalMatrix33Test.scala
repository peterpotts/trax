package com.peterpotts.trax.math

import com.peterpotts.example.RandomExample._
import com.peterpotts.trax.math.GeometricEquality._
import com.peterpotts.trax.math.GeometryExamples._
import org.scalatest.matchers.should.Matchers
import org.scalatest.wordspec.AnyWordSpec

/**
  * @author Peter Potts
  */
class DiagonalMatrix33Test extends AnyWordSpec with Matchers {
  "Diagonal Matrix 3 by 3" should {
    "evaluate multiply scalar" in {
      val a = exampleDiagonalMatrix33.next()
      val b = exampleDouble.next()
      val c = a.matrix33 * b
      a * b shouldEqual c
    }

    "evaluate multiply vector" in {
      val a = exampleDiagonalMatrix33.next()
      val b = exampleVector3.next()
      val c = a.matrix33 * b
      a * b shouldEqual c
    }

    "evaluate multiply 3 by 3 matrix" in {
      val a = exampleDiagonalMatrix33.next()
      val b = exampleMatrix33.next()
      val c = a.matrix33 * b
      a * b shouldEqual c
    }

    "evaluate divide scalar" in {
      val a = exampleDiagonalMatrix33.next()
      val b = exampleDouble.next()
      val c = a.matrix33 / b
      a / b shouldEqual c
    }

    "evaluate inverse" in {
      val a = exampleDiagonalMatrix33.next()
      val b = a.matrix33.inverse
      a.inverse.matrix33 shouldEqual b
    }

    "evaluate squared" in {
      val a = exampleDiagonalMatrix33.next()
      val b = a.matrix33 * a.matrix33
      a.squared.matrix33 shouldEqual b
    }
  }
}
