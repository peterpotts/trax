package com.peterpotts.trax.kalman

import com.peterpotts.trax.math.VectorDDecorator._
import com.peterpotts.trax.models.a.ModelA
import org.scalatest.matchers.should.Matchers
import org.scalatest.wordspec.AnyWordSpec

/**
  * @author Peter Potts
  */
class ModelATest extends AnyWordSpec with Matchers {
  "Model A Kalman filter" should {
    "predict and update" in {
      val last = ModelA.steps.last
      val error = (last.target.vectorD - last.estimate.x.vectorD).norm
      println("error = " + error)
      error shouldBe <=(1.0)
    }
  }
}
