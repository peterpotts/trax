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
class Matrix33Test extends AnyWordSpec with Matchers {
  "Matrix 3 by 3" should {
    "evaluate add" in {
      val a = exampleMatrix33.next()
      val b = exampleMatrix33.next()
      val c = (a.matrixD + b.matrixD).matrix33
      a + b shouldEqual c
    }

    "evaluate subtract" in {
      val a = exampleMatrix33.next()
      val b = exampleMatrix33.next()
      val c = (a.matrixD - b.matrixD).matrix33
      a - b shouldEqual c
    }

    "evaluate multiply scalar" in {
      val a = exampleMatrix33.next()
      val b = exampleDouble.next()
      val c = (a.matrixD * b).matrix33
      a * b shouldEqual c
    }

    "evaluate multiply vector" in {
      val a = exampleMatrix33.next()
      val b = exampleVector3.next()
      val c = (a.matrixD ** b.vectorD).vector3
      a * b shouldEqual c
    }

    "evaluate multiply 3 by 3 matrix" in {
      val a = exampleMatrix33.next()
      val b = exampleMatrix33.next()
      val c = (a.matrixD *** b.matrixD).matrix33
      a * b shouldEqual c
    }

    "evaluate multiply 3 by 4 matrix" in {
      val a = exampleMatrix33.next()
      val b = exampleMatrix34.next()
      val c = (a.matrixD *** b.matrixD).matrix34
      a * b shouldEqual c
    }

    "evaluate divide scalar" in {
      val a = exampleMatrix33.next()
      val b = exampleDouble.next()
      val c = (a.matrixD / b).matrix33
      a / b shouldEqual c
    }

    "evaluate cofactors" in {
      val a = exampleMatrix33.next()
      a.cxx shouldEqual a.matrixD.cofactor(0, 0)
      a.cxy shouldEqual a.matrixD.cofactor(0, 1)
      a.cxz shouldEqual a.matrixD.cofactor(0, 2)
      a.cyx shouldEqual a.matrixD.cofactor(1, 0)
      a.cyy shouldEqual a.matrixD.cofactor(1, 1)
      a.cyz shouldEqual a.matrixD.cofactor(1, 2)
      a.czx shouldEqual a.matrixD.cofactor(2, 0)
      a.czy shouldEqual a.matrixD.cofactor(2, 1)
      a.czz shouldEqual a.matrixD.cofactor(2, 2)
    }

    "evaluate determinant" in {
      val a = exampleMatrix33.next()
      a.determinant shouldEqual a.matrixD.determinant
    }

    "evaluate transpose" in {
      val a = exampleMatrix33.next()
      val b = a.matrixD.transpose.matrix33
      a.transpose shouldEqual b
    }

    "evaluate inverse" in {
      val a = exampleMatrix33.next()
      val b = a.matrixD.inverse.matrix33
      a.inverse shouldEqual b
    }

    "satisfy identity property for multiplication property" in {
      val a = exampleMatrix33.next()
      a * Matrix33.identity shouldEqual a
    }

    "satisfy transpose identity to identity property" in {
      Matrix33.identity.transpose shouldEqual Matrix33.identity
    }

    "satisfy associative property for multiplication" in {
      val a = exampleMatrix33.next()
      val b = exampleMatrix33.next()
      val c = exampleMatrix33.next()
      (a * b) * c shouldEqual (a * (b * c))
    }

    "satisfy transitive property for multiplication and addition" in {
      val a = exampleMatrix33.next()
      val b = exampleMatrix33.next()
      val c = exampleMatrix33.next()
      (a + b) * c shouldEqual (a * c) + (b * c)
    }

    "satisfy inverse property for multiplication" in {
      val a = exampleMatrix33.next()
      a.inverse * a shouldEqual Matrix33.identity
      a * a.inverse shouldEqual Matrix33.identity
    }

    "satisfy transpose property for multiplication" in {
      val a = exampleMatrix33.next()
      val b = exampleMatrix33.next()
      a.transpose * b.transpose shouldEqual (b * a).transpose
    }
  }
}
