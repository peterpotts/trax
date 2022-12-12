package com.peterpotts.trax.math

import com.peterpotts.trax.math.GeometricEquality._
import org.scalatest.matchers.should.Matchers
import org.scalatest.wordspec.AnyWordSpec

/**
  * @author Peter Potts
  */
class IntegrationTest extends AnyWordSpec with Matchers {
  "A body-fixed angular velocity" should {
    "integrate beta on left hand side once" in {
      val eulerAngles = EulerAngles(ϕ = 0, θ = 45.toRadians, ψ = 0)
      val beta = Vector3(1.toRadians, 0, 0)
      val expected = EulerAngles(ϕ = 1.toRadians, θ = 45.toRadians, ψ = 0)
      val actual = (RotationVector(beta).unitQuaternion ∘ eulerAngles.unitQuaternion).eulerAngles
      actual shouldEqual expected
    }

    "integrate beta on left hand side twice" in {
      val eulerAngles = EulerAngles(ϕ = 90.toRadians, θ = 0, ψ = 0)
      val beta = Vector3(0, 1.toRadians, 0)
      val expected = EulerAngles(ϕ = 90.toRadians, θ = 0, ψ = 1.toRadians)
      val actual = (RotationVector(beta).unitQuaternion ∘ eulerAngles.unitQuaternion).eulerAngles
      actual shouldEqual expected
    }
  }
}
