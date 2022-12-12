package com.peterpotts.trax.kalman

import com.peterpotts.trax.math.VectorDDecorator._
import com.peterpotts.trax.models.b.ModelB
import org.scalatest.matchers.should.Matchers
import org.scalatest.wordspec.AnyWordSpec

/**
  * @author Peter Potts
  */
class ModelBTest extends AnyWordSpec with Matchers {
  "Model B Kalman filter" should {
    "predict and update" ignore {
      val last = ModelB.steps.last
      val error = (last.target.vectorD - last.estimate.x.vectorD).norm
      println("error = " + error)
      error shouldBe <=(1.0)
    }
  }
}
