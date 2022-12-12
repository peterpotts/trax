package com.peterpotts.trax.math

import com.peterpotts.example.RandomExample._
import com.peterpotts.trax.math.GeometricEquality._
import com.peterpotts.trax.math.GeometryExamples._
import com.peterpotts.trax.math.TensorDDecorator._
import org.scalatest.matchers.should.Matchers
import org.scalatest.wordspec.AnyWordSpec

/**
  * @author Peter Potts
  */
class TensorDTest extends AnyWordSpec with Matchers {
  "Tensor D" should {
    "evaluate is" in {
      exampleTensor334.next().tensorD.is(3, 3, 4) shouldEqual true
    }

    "evaluate rows" in {
      exampleTensor334.next().tensorD.rows shouldEqual Vector(0, 1, 2)
    }

    "evaluate columns" in {
      exampleTensor334.next().tensorD.columns shouldEqual Vector(0, 1, 2)
    }

    "evaluate layers" in {
      exampleTensor334.next().tensorD.layers shouldEqual Vector(0, 1, 2, 3)
    }

    "evaluate add" in {
      val a = exampleTensor334.next().tensorD
      val b = exampleTensor334.next().tensorD
      val c = Vector.tabulate[Double](3, 3, 4) { (i, j, k) => a(i)(j)(k) + b(i)(j)(k) }
      a + b shouldEqual c
    }

    "evaluate subtract" in {
      val a = exampleTensor334.next().tensorD
      val b = exampleTensor334.next().tensorD
      val c = Vector.tabulate[Double](3, 3, 4) { (i, j, k) => a(i)(j)(k) - b(i)(j)(k) }
      a - b shouldEqual c
    }

    "evaluate multiply scalar" in {
      val a = exampleTensor334.next().tensorD
      val b = exampleDouble.next()
      val c = Vector.tabulate[Double](3, 3, 4) { (i, j, k) => a(i)(j)(k) * b }
      a * b shouldEqual c
    }

    "evaluate multiply vector" in {
      val a = exampleTensor334.next().tensorD
      val b = exampleVector4.next().vectorD

      val c = Vector.tabulate[Double](3, 3) { (i, j) =>
        val elements = for (k <- 0 until 4) yield a(i)(j)(k) * b(k)
        elements.sum
      }

      a ** b shouldEqual c
    }

    "evaluate multiply matrix" in {
      val a = exampleTensor334.next().tensorD
      val b = exampleMatrix43.next().matrixD

      val c = Vector.tabulate[Double](3, 3, 3) { (i, j, l) =>
        val elements = for (k <- 0 until 4) yield a(i)(j)(k) * b(k)(l)
        elements.sum
      }

      a *** b shouldEqual c
    }

    "evaluate IJ transpose" in {
      val a = exampleTensor334.next().tensorD
      val b = Vector.tabulate[Double](3, 3, 4) { (i, j, k) => a(j)(i)(k) }
      a.transposeIJ shouldEqual b
    }

    "evaluate JK transpose" in {
      val a = exampleTensor334.next().tensorD
      val b = Vector.tabulate[Double](3, 4, 3) { (i, j, k) => a(i)(k)(j) }
      a.transposeJK shouldEqual b
    }

    "evaluate IK transpose" in {
      val a = exampleTensor334.next().tensorD
      val b = Vector.tabulate[Double](4, 3, 3) { (i, j, k) => a(k)(j)(i) }
      a.transposeIK shouldEqual b
    }
  }
}
