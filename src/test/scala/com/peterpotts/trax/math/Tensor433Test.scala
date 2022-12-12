package com.peterpotts.trax.math

import com.peterpotts.example.RandomExample._
import com.peterpotts.trax.math.GeometricEquality._
import com.peterpotts.trax.math.GeometryExamples._
import com.peterpotts.trax.math.MatrixDDecorator._
import com.peterpotts.trax.math.TensorDDecorator._
import org.scalatest.matchers.should.Matchers
import org.scalatest.wordspec.AnyWordSpec

/**
  * @author Peter Potts
  */
class Tensor433Test extends AnyWordSpec with Matchers {
  "Tensor 4 by 3 by 3" should {
    "evaluate add" in {
      val a = exampleTensor433.next()
      val b = exampleTensor433.next()
      val c = (a.tensorD + b.tensorD).tensor433
      a + b shouldEqual c
    }

    "evaluate subtract" in {
      val a = exampleTensor433.next()
      val b = exampleTensor433.next()
      val c = (a.tensorD - b.tensorD).tensor433
      a - b shouldEqual c
    }

    "evaluate multiply scalar" in {
      val a = exampleTensor433.next()
      val b = exampleDouble.next()
      val c = (a.tensorD * b).tensor433
      a * b shouldEqual c
    }

    "evaluate multiply vector" in {
      val a = exampleTensor433.next()
      val b = exampleVector3.next()
      val c = (a.tensorD ** b.vectorD).matrix43
      a * b shouldEqual c
    }

    "evaluate multiply 3 by 3 matrix" in {
      val a = exampleTensor433.next()
      val b = exampleMatrix33.next()
      val c = (a.tensorD *** b.matrixD).tensor433
      a * b shouldEqual c
    }

    "evaluate divide scalar" in {
      val a = exampleTensor433.next()
      val b = exampleDouble.next()
      val c = (a.tensorD / b).tensor433
      a / b shouldEqual c
    }

    "evaluate IJ transpose" in {
      val a = exampleTensor433.next()
      val b = a.tensorD.transposeIJ.tensor343
      a.transposeIJ shouldEqual b
    }

    "evaluate JK transpose" in {
      val a = exampleTensor433.next()
      val b = a.tensorD.transposeJK.tensor433
      a.transposeJK shouldEqual b
    }

    "evaluate IK transpose" in {
      val a = exampleTensor433.next()
      val b = a.tensorD.transposeIK.tensor334
      a.transposeIK shouldEqual b
    }
  }
}
