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
class Tensor333Test extends AnyWordSpec with Matchers {
  "Tensor 3 by 3 by 3" should {
    "evaluate add" in {
      val a = exampleTensor333.next()
      val b = exampleTensor333.next()
      val c = (a.tensorD + b.tensorD).tensor333
      a + b shouldEqual c
    }

    "evaluate subtract" in {
      val a = exampleTensor333.next()
      val b = exampleTensor333.next()
      val c = (a.tensorD - b.tensorD).tensor333
      a - b shouldEqual c
    }

    "evaluate multiply scalar" in {
      val a = exampleTensor333.next()
      val b = exampleDouble.next()
      val c = (a.tensorD * b).tensor333
      a * b shouldEqual c
    }

    "evaluate multiply vector" in {
      val a = exampleTensor333.next()
      val b = exampleVector3.next()
      val c = (a.tensorD ** b.vectorD).matrix33
      a * b shouldEqual c
    }

    "evaluate divide scalar" in {
      val a = exampleTensor333.next()
      val b = exampleDouble.next()
      val c = (a.tensorD / b).tensor333
      a / b shouldEqual c
    }

    "evaluate IJ transpose" in {
      val a = exampleTensor333.next()
      val b = a.tensorD.transposeIJ.tensor333
      a.transposeIJ shouldEqual b
    }

    "evaluate JK transpose" in {
      val a = exampleTensor333.next()
      val b = a.tensorD.transposeJK.tensor333
      a.transposeJK shouldEqual b
    }

    "evaluate IK transpose" in {
      val a = exampleTensor333.next()
      val b = a.tensorD.transposeIK.tensor333
      a.transposeIK shouldEqual b
    }
  }
}
