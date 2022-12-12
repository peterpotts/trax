package com.peterpotts.trax.math

import com.peterpotts.example.RandomExample._
import com.peterpotts.trax.math.GeometricEquality._
import com.peterpotts.trax.math.GeometryExamples._
import org.scalatest.matchers.should.Matchers
import org.scalatest.wordspec.AnyWordSpec

/**
  * @author Peter Potts
  */
class EulerAnglesTest extends AnyWordSpec with Matchers {
  "Euler angles" should {
    "roll" in {
      val roll = exampleAngle.next()
      EulerAngles(ϕ = roll, θ = 0, ψ = 0).rotationMatrix shouldEqual RotationMatrix33.X(roll)
    }

    "pitch" in {
      val pitch = exampleAngle.next()
      EulerAngles(ϕ = 0, θ = pitch, ψ = 0).rotationMatrix shouldEqual RotationMatrix33.Y(pitch)
    }

    "yaw" in {
      val yaw = exampleAngle.next()
      EulerAngles(ϕ = 0, θ = 0, ψ = yaw).rotationMatrix shouldEqual RotationMatrix33.Z(yaw)
    }

    "yaw and pitch" in {
      EulerAngles(ϕ = 0, θ = 90.toRadians, ψ = 90.toRadians).rotationMatrix shouldEqual
        RotationMatrix33(Matrix33(Vector3.Z.negate, Vector3.X.negate, Vector3.Y))
    }

    "yaw and pitch and roll" in {
      EulerAngles(ϕ = 90.toRadians, θ = 90.toRadians, ψ = 90.toRadians).rotationMatrix shouldEqual
        RotationMatrix33(Matrix33(Vector3.Z.negate, Vector3.Y, Vector3.X))
    }

    "pitch and roll" in {
      EulerAngles(ϕ = 90.toRadians, θ = 90.toRadians, ψ = 0).rotationMatrix shouldEqual
        RotationMatrix33(Matrix33(Vector3.Z.negate, Vector3.X, Vector3.Y.negate))
    }

    "have identity equivalent to quaternion identity" in {
      EulerAngles.zero.unitQuaternion shouldEqual UnitQuaternion.zero
    }

    "represent roll, pitch and yaw" in {
      val eulerAngles = exampleEulerAngles.next()

      val rotationMatrix =
        RotationMatrix33.X(eulerAngles.ϕ) *
          RotationMatrix33.Y(eulerAngles.θ) *
          RotationMatrix33.Z(eulerAngles.ψ)

      eulerAngles.rotationMatrix shouldEqual rotationMatrix
    }

    "has invariant π transforms" in {
      val eulerAngles = exampleEulerAngles.next()
      import eulerAngles.{θ, ψ, ϕ}
      eulerAngles shouldEqual EulerAngles(ϕ + 2 * π, θ, ψ, validate = false).unitQuaternion.eulerAngles
      eulerAngles shouldEqual EulerAngles(ϕ - 2 * π, θ, ψ, validate = false).unitQuaternion.eulerAngles
      eulerAngles shouldEqual EulerAngles(ϕ, θ + 2 * π, ψ, validate = false).unitQuaternion.eulerAngles
      eulerAngles shouldEqual EulerAngles(ϕ, θ - 2 * π, ψ, validate = false).unitQuaternion.eulerAngles
      eulerAngles shouldEqual EulerAngles(ϕ, θ, ψ + 2 * π, validate = false).unitQuaternion.eulerAngles
      eulerAngles shouldEqual EulerAngles(ϕ, θ, ψ - 2 * π, validate = false).unitQuaternion.eulerAngles
      eulerAngles shouldEqual EulerAngles(ϕ + π, -π - θ, ψ + π, validate = false).unitQuaternion.eulerAngles
      eulerAngles shouldEqual EulerAngles(ϕ + π, π - θ, ψ + π, validate = false).unitQuaternion.eulerAngles
    }

    "normalize" in {
      val roll = examplePlusMinusFourPi.next()
      val pitch = examplePlusMinusFourPi.next()
      val yaw = examplePlusMinusFourPi.next()
      val eulerAngles = EulerAngles(roll, pitch, yaw, validate = false)
      EulerAngles.normalize(roll, pitch, yaw) shouldEqual eulerAngles.unitQuaternion.eulerAngles
    }

    "have optimal inverse conjugate rates matrix" in {
      val eulerAngles = exampleEulerAngles.next()
      eulerAngles.inverseConjugateRatesMatrix shouldEqual eulerAngles.conjugateRatesMatrix.inverse
    }

    "reversible with unit quaternion" in {
      val eulerAngles = exampleEulerAngles.next()

      if (!eulerAngles.isGimbalLocked)
        eulerAngles.unitQuaternion.eulerAngles shouldEqual eulerAngles
    }

    "reversible with axis-angle" in {
      val eulerAngles = exampleEulerAngles.next()

      if (!eulerAngles.isGimbalLocked)
        eulerAngles.unitQuaternion.axisAngle.unitQuaternion.eulerAngles shouldEqual eulerAngles
    }

    "reversible with rotation vector" in {
      val eulerAngles = exampleEulerAngles.next()

      if (!eulerAngles.isGimbalLocked)
        eulerAngles.unitQuaternion.rotationVector.unitQuaternion.eulerAngles shouldEqual eulerAngles
    }

    /**
      * Page 9 (43, 44)
      */
    "satisfy rates and rotation matrix property" in {
      val eulerAngles = exampleEulerAngles.next()

      eulerAngles.rotationMatrix.matrix33 shouldEqual
        eulerAngles.conjugateRatesMatrix * eulerAngles.ratesMatrix.inverse

      eulerAngles.rotationMatrix.inverse.matrix33 shouldEqual
        eulerAngles.ratesMatrix * eulerAngles.conjugateRatesMatrix.inverse
    }
  }
}
