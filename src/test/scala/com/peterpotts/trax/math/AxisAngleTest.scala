package com.peterpotts.trax.math

import com.peterpotts.trax.math.GeometricEquality._
import com.peterpotts.trax.math.GeometryExamples._
import org.scalatest.matchers.should.Matchers
import org.scalatest.wordspec.AnyWordSpec

class AxisAngleTest extends AnyWordSpec with Matchers {
  "Axis-angle" should {
    "invertible" in {
      val axisAngle = exampleAxisAngle.next()
      axisAngle.inverse.rotationMatrix shouldEqual axisAngle.rotationMatrix.inverse
    }

    "rotate X axis by 30 degrees about X axis" in {
      AxisAngle(Vector3.X, 30.toRadians).rotationMatrix shouldEqual RotationMatrix33.X(30.toRadians)
    }

    "optimize rotation about X axis" in {
      val angle = examplePlusMinusTwoPi.next()
      RotationMatrix33.X(angle) shouldEqual AxisAngle.normalize(Vector3.X, angle).rotationMatrix
    }

    "optimize rotation about Y axis" in {
      val angle = examplePlusMinusTwoPi.next()
      RotationMatrix33.Y(angle) shouldEqual AxisAngle.normalize(Vector3.Y, angle).rotationMatrix
    }

    "optimize rotation about Z axis" in {
      val angle = examplePlusMinusTwoPi.next()
      RotationMatrix33.Z(angle) shouldEqual AxisAngle.normalize(Vector3.Z, angle).rotationMatrix
    }

    "reversible with rotation vector" in {
      val axisAngle = exampleAxisAngle.next()
      axisAngle.rotationVector.axisAngle shouldEqual axisAngle
    }

    "reversible with unit quaternion" in {
      val axisAngle = exampleAxisAngle.next()
      axisAngle.unitQuaternion.axisAngle shouldEqual axisAngle
    }

    "reversible with Euler angle" in {
      val axisAngle = exampleAxisAngle.next()

      if (!axisAngle.unitQuaternion.eulerAngles.isGimbalLocked)
        axisAngle.unitQuaternion.eulerAngles.unitQuaternion.axisAngle shouldEqual axisAngle
    }
  }
}
