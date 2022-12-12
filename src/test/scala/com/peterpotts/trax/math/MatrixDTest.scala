package com.peterpotts.trax.math

import com.peterpotts.example.RandomExample._
import com.peterpotts.trax.math.GeometricEquality._
import com.peterpotts.trax.math.GeometryExamples._
import com.peterpotts.trax.math.MatrixDDecorator._
import org.scalatest.matchers.should.Matchers
import org.scalatest.wordspec.AnyWordSpec

/**
  * @author Peter Potts
  */
class MatrixDTest extends AnyWordSpec with Matchers {
  "Matrix D" should {
    "append horizontally" in {
      MatrixD.horizontal(
        Matrix22(Vector2(1, 2), Vector2(3, 4)),
        Matrix22(Vector2(5, 6), Vector2(7, 8))
      ) shouldEqual MatrixD(VectorD(1, 2, 5, 6), VectorD(3, 4, 7, 8))

      MatrixD.horizontal(Matrix12(Vector2(1, 2)), Matrix12(Vector2(3, 4))) shouldEqual
        MatrixD(VectorD(1, 2, 3, 4))
    }

    "append vertically" in {
      MatrixD.vertical(
        MatrixD(VectorD(1, 2), VectorD(3, 4)),
        MatrixD(VectorD(5, 6), VectorD(7, 8))
      ) shouldEqual MatrixD(VectorD(1, 2), VectorD(3, 4), VectorD(5, 6), VectorD(7, 8))

      MatrixD.vertical(MatrixD(VectorD(1, 2)), MatrixD(VectorD(3, 4))) shouldEqual
        MatrixD(VectorD(1, 2), VectorD(3, 4))
    }

    "evaluate is" in {
      exampleMatrix43.next().matrixD.is(4, 3) shouldEqual true
    }

    "evaluate rows" in {
      exampleMatrix43.next().matrixD.rows shouldEqual Vector(0, 1, 2, 3)
    }

    "evaluate columns" in {
      exampleMatrix43.next().matrixD.columns shouldEqual Vector(0, 1, 2)
    }

    "evaluate add" in {
      val a = exampleMatrix43.next().matrixD
      val b = exampleMatrix43.next().matrixD
      val c = Vector.tabulate[Double](4, 3) { (i, j) => a(i)(j) + b(i)(j) }
      a + b shouldEqual c
    }

    "evaluate subtract" in {
      val a = exampleMatrix43.next().matrixD
      val b = exampleMatrix43.next().matrixD
      val c = Vector.tabulate[Double](4, 3) { (i, j) => a(i)(j) - b(i)(j) }
      a - b shouldEqual c
    }

    "evaluate multiply scalar" in {
      val a = exampleMatrix43.next().matrixD
      val b = exampleDouble.next()
      val c = Vector.tabulate[Double](4, 3) { (i, j) => a(i)(j) * b }
      a * b shouldEqual c
    }

    "evaluate multiply vector" in {
      val a = exampleMatrix43.next().matrixD
      val b = exampleVector3.next().vectorD

      val c = Vector.tabulate[Double](4) { i =>
        val elements = for (j <- 0 until 3) yield a(i)(j) * b(j)
        elements.sum
      }

      a ** b shouldEqual c
    }

    "evaluate multiply matrix" in {
      val a = exampleMatrix43.next().matrixD
      val b = exampleMatrix34.next().matrixD

      val c = Vector.tabulate[Double](4, 4) { (i, k) =>
        val elements = for (j <- 0 until 3) yield a(i)(j) * b(j)(k)
        elements.sum
      }

      a *** b shouldEqual c
    }

    "evaluate divide scalar" in {
      val a = exampleMatrix43.next().matrixD
      val b = exampleDouble.next()
      val c = Vector.tabulate[Double](4, 3) { (i, j) => a(i)(j) / b }
      a / b shouldEqual c
    }
  }

  "evaluate cofactors" in {
    val a = MatrixD(VectorD(5, 2, 1), VectorD(4, 7, 3), VectorD(9, 6, 8))
    a.cofactor(0, 0) shouldEqual 38.0
    a.cofactor(0, 1) shouldEqual -5.0
    a.cofactor(0, 2) shouldEqual -39.0
    a.cofactor(1, 0) shouldEqual -10.0
    a.cofactor(1, 1) shouldEqual 31.0
    a.cofactor(1, 2) shouldEqual -12.0
    a.cofactor(2, 0) shouldEqual -1.0
    a.cofactor(2, 1) shouldEqual -11.0
    a.cofactor(2, 2) shouldEqual 27.0
  }

  "evaluate submatrix" in {
    MatrixD(VectorD(1, 2, 3), VectorD(4, 5, 6), VectorD(7, 8, 9)).submatrix(1, 1) shouldEqual
      MatrixD(VectorD(1, 3), VectorD(7, 9))
  }

  "evaluate determinant" in {
    MatrixD(VectorD(5, 2, 1), VectorD(4, 7, 3), VectorD(9, 6, 8)).determinant shouldEqual 141.0
  }

  "evaluate transpose" in {
    val a = exampleMatrix43.next().matrixD
    val b = Vector.tabulate[Double](3, 4) { (i, j) => a(j)(i) }
    a.transpose shouldEqual b
  }

  "evaluate inverse" in {
    val a = MatrixD(VectorD(1, 4, 7), VectorD(3, 0, 5), VectorD(-1, 9, 11))
    val b = MatrixD(VectorD(5.625, -2.375, -2.5), VectorD(4.75, -2.25, -2), VectorD(-3.375, 1.625, 1.5))
    a.inverse shouldEqual b
  }

  "evaluate slow and fast inverse" in {
    val a = exampleMatrixD(6, 6).next()
    a.inverse shouldEqual a.slowInverse
  }

  "satisfy identity property for multiplication property" in {
    val a = exampleMatrixD(3, 3).next()
    a *** MatrixD.identity(3) shouldEqual a
  }

  "satisfy transpose identity to identity property" in {
    MatrixD.identity(3).transpose shouldEqual MatrixD.identity(3)
  }

  "satisfy associative property for multiplication" in {
    val a = exampleMatrixD(3, 3).next()
    val b = exampleMatrixD(3, 3).next()
    val c = exampleMatrixD(3, 3).next()
    (a *** b) *** c shouldEqual (a *** (b *** c))
  }

  "satisfy transitive property for multiplication and addition" in {
    val a = exampleMatrixD(3, 3).next()
    val b = exampleMatrixD(3, 3).next()
    val c = exampleMatrixD(3, 3).next()
    (a + b) *** c shouldEqual (a *** c) + (b *** c)
  }

  "satisfy inverse property for multiplication" in {
    val a = exampleMatrixD(20, 20).next()
    a.inverse *** a shouldEqual MatrixD.identity(3)
    a *** a.inverse shouldEqual MatrixD.identity(3)
  }
}
