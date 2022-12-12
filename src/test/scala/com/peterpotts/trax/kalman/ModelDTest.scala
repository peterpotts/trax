package com.peterpotts.trax.kalman

import com.peterpotts.trax.math.VectorDDecorator._
import com.peterpotts.trax.models.d.ModelD
import org.scalatest.matchers.should.Matchers
import org.scalatest.wordspec.AnyWordSpec

/**
  * @author Peter Potts
  */
class ModelDTest extends AnyWordSpec with Matchers {
  "Model D extended Kalman filter" should {
    "predict and update" in {
      val last = ModelD.steps.last
      val error = (last.target.vectorD - last.estimate.x.vectorD).norm
      println("error = " + error)
      error shouldBe <=(1.0)
    }
  }
}
