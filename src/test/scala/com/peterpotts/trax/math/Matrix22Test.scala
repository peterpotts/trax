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
class Matrix22Test extends AnyWordSpec with Matchers {
  "Matrix 2 by 2" should {
    "dot" in {
      val a = Matrix22(Vector2(1, 2), Vector2(3, 4))
      val b = Matrix22(Vector2(5, 6), Vector2(7, 8))
      val c = Matrix22(Vector2(19, 22), Vector2(43, 50))
      a * b shouldEqual c
    }

    "evaluate add" in {
      val a = exampleMatrix22.next()
      val b = exampleMatrix22.next()
      val c = (a.matrixD + b.matrixD).matrix22
      a + b shouldEqual c
    }

    "evaluate subtract" in {
      val a = exampleMatrix22.next()
      val b = exampleMatrix22.next()
      val c = (a.matrixD - b.matrixD).matrix22
      a - b shouldEqual c
    }

    "evaluate multiply scalar" in {
      val a = exampleMatrix22.next()
      val b = exampleDouble.next()
      val c = (a.matrixD * b).matrix22
      a * b shouldEqual c
    }

    "evaluate multiply vector" in {
      val a = exampleMatrix22.next()
      val b = exampleVector2.next()
      val c = (a.matrixD ** b.vectorD).vector2
      a * b shouldEqual c
    }

    "evaluate multiply 2 by 2 matrix" in {
      val a = exampleMatrix22.next()
      val b = exampleMatrix22.next()
      val c = (a.matrixD *** b.matrixD).matrix22
      a * b shouldEqual c
    }

    "evaluate divide scalar" in {
      val a = exampleMatrix22.next()
      val b = exampleDouble.next()
      val c = (a.matrixD / b).matrix22
      a / b shouldEqual c
    }

    "evaluate cofactors" in {
      val a = exampleMatrix22.next()
      a.cxx shouldEqual a.matrixD.cofactor(0, 0)
      a.cxy shouldEqual a.matrixD.cofactor(0, 1)
      a.cyx shouldEqual a.matrixD.cofactor(1, 0)
      a.cyy shouldEqual a.matrixD.cofactor(1, 1)
    }

    "evaluate determinant" in {
      val a = exampleMatrix22.next()
      a.determinant shouldEqual a.matrixD.determinant
    }

    "evaluate transpose" in {
      val a = exampleMatrix22.next()
      val b = a.matrixD.transpose.matrix22
      a.transpose shouldEqual b
    }

    "evaluate inverse" in {
      val a = exampleMatrix22.next()
      val b = a.matrixD.inverse.matrix22
      a.inverse shouldEqual b
    }

    "satisfy identity property for multiplication property" in {
      val a = exampleMatrix22.next()
      a * Matrix22.identity shouldEqual a
    }

    "satisfy transpose identity to identity property" in {
      Matrix22.identity.transpose shouldEqual Matrix22.identity
    }

    "satisfy associative property for multiplication" in {
      val a = exampleMatrix22.next()
      val b = exampleMatrix22.next()
      val c = exampleMatrix22.next()
      (a * b) * c shouldEqual (a * (b * c))
    }

    "satisfy transitive property for multiplication and addition" in {
      val a = exampleMatrix22.next()
      val b = exampleMatrix22.next()
      val c = exampleMatrix22.next()
      (a + b) * c shouldEqual (a * c) + (b * c)
    }

    "satisfy inverse property for multiplication" in {
      val a = exampleMatrix22.next()
      a.inverse * a shouldEqual Matrix22.identity
      a * a.inverse shouldEqual Matrix22.identity
    }

    "satisfy transpose property for multiplication" in {
      val a = exampleMatrix22.next()
      val b = exampleMatrix22.next()
      a.transpose * b.transpose shouldEqual (b * a).transpose
    }
  }
}
