package com.peterpotts.trax.math

import com.peterpotts.example.RandomExample._
import com.peterpotts.trax.math.GeometricEquality._
import com.peterpotts.trax.math.GeometryExamples._
import org.scalatest.matchers.should.Matchers
import org.scalatest.wordspec.AnyWordSpec

/**
  * @author Peter Potts
  */
class RotationVectorTest extends AnyWordSpec with Matchers {
  "A rotation vector" should {
    "reversible with axis-angle" in {
      val rotationVector = exampleRotationVector.next()
      rotationVector.axisAngle.rotationVector shouldEqual rotationVector
    }

    "reversible with unit quaternion" in {
      val rotationVector = exampleRotationVector.next()
      rotationVector.unitQuaternion.rotationVector shouldEqual rotationVector
    }

    "reversible with Euler angle" in {
      val rotationVector = exampleRotationVector.next()

      if (!rotationVector.unitQuaternion.eulerAngles.isGimbalLocked)
        rotationVector.unitQuaternion.eulerAngles.unitQuaternion.rotationVector shouldEqual rotationVector
    }

    "invertible" in {
      val rotationVector = exampleRotationVector.next()
      rotationVector.inverse.rotationMatrix shouldEqual rotationVector.rotationMatrix.inverse
    }

    "orientate" in {
      // Orientation of fixed-body relative to world
      val r = exampleRotationVector.next()

      // Reorientation of reoriented world
      val s = exampleRotationVector.next()

      // Orientation of fixed-body relative to reoriented world
      val ro = (r.unitQuaternion ∘ s.inverse.unitQuaternion).rotationVector

      r shouldEqual (ro.unitQuaternion ∘ s.unitQuaternion).rotationVector

      // Vector in world coordinates
      val z = exampleVector3.next()
      // Vector in fixed-body coordinates
      val zb = r.rotationMatrix.matrix33 * z
      // Vector in reoriented world coordinates
      val zo = s.rotationMatrix.matrix33 * z

      // Transform from world to fixed-body
      (r.rotationMatrix.matrix33 * z) shouldEqual zb
      // Transform from fixed-body to world
      (r.rotationMatrix.inverse.matrix33 * zb) shouldEqual z

      // Transform from reoriented world to fixed-body
      (ro.rotationMatrix.matrix33 * zo) shouldEqual zb
      // Transform from fixed-body to reoriented world
      (ro.rotationMatrix.inverse.matrix33 * zb) shouldEqual zo
    }
  }
}
