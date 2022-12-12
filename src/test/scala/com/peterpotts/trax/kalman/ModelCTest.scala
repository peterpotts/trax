package com.peterpotts.trax.kalman

import com.peterpotts.trax.math.VectorDDecorator._
import com.peterpotts.trax.models.c.ModelC
import org.scalatest.matchers.should.Matchers
import org.scalatest.wordspec.AnyWordSpec

/**
  * @author Peter Potts
  */
class ModelCTest extends AnyWordSpec with Matchers {
  "Model C Kalman filter" should {
    "predict and update" in {
      val last = ModelC.steps.last
      val error = (last.target.vectorD - last.estimate.x.vectorD).norm
      println("error = " + error)
      error shouldBe <=(1.0)
    }
  }
}
