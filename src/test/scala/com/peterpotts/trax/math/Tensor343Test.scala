package com.peterpotts.trax.math

import com.peterpotts.example.RandomExample._
import com.peterpotts.trax.math.GeometryExamples._
import com.peterpotts.trax.math.MatrixDDecorator._
import com.peterpotts.trax.math.TensorDDecorator._
import org.scalatest.matchers.should.Matchers
import org.scalatest.wordspec.AnyWordSpec

/**
  * @author Peter Potts
  */
class Tensor343Test extends AnyWordSpec with Matchers {
  "Tensor 3 by 3 by 4" should {
    "evaluate add" in {
      val a = exampleTensor343.next()
      val b = exampleTensor343.next()
      val c = (a.tensorD + b.tensorD).tensor343
      a + b shouldEqual c
    }

    "evaluate subtract" in {
      val a = exampleTensor343.next()
      val b = exampleTensor343.next()
      val c = (a.tensorD - b.tensorD).tensor343
      a - b shouldEqual c
    }

    "evaluate multiply scalar" in {
      val a = exampleTensor343.next()
      val b = exampleDouble.next()
      val c = (a.tensorD * b).tensor343
      a * b shouldEqual c
    }

    "evaluate multiply vector" in {
      val a = exampleTensor343.next()
      val b = exampleVector3.next()
      val c = (a.tensorD ** b.vectorD).matrix34
      a * b shouldEqual c
    }

    "evaluate multiply 3 by 3 matrix" in {
      val a = exampleTensor343.next()
      val b = exampleMatrix33.next()
      val c = (a.tensorD *** b.matrixD).tensor343
      a * b shouldEqual c
    }

    "evaluate divide scalar" in {
      val a = exampleTensor343.next()
      val b = exampleDouble.next()
      val c = (a.tensorD / b).tensor343
      a / b shouldEqual c
    }

    "evaluate IJ transpose" in {
      val a = exampleTensor343.next()
      val b = a.tensorD.transposeIJ.tensor433
      a.transposeIJ shouldEqual b
    }

    "evaluate JK transpose" in {
      val a = exampleTensor343.next()
      val b = a.tensorD.transposeJK.tensor334
      a.transposeJK shouldEqual b
    }

    "evaluate IK transpose" in {
      val a = exampleTensor343.next()
      val b = a.tensorD.transposeIK.tensor343
      a.transposeIK shouldEqual b
    }
  }
}
