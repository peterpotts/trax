package com.peterpotts.trax.math

import com.peterpotts.example.RandomExample._
import com.peterpotts.trax.math.GeometricEquality._
import com.peterpotts.trax.math.GeometryExamples._
import com.peterpotts.trax.math.MathDecorator._
import org.scalatest.matchers.should.Matchers
import org.scalatest.wordspec.AnyWordSpec

/**
  * @author Peter Potts
  */
class UnitQuaternionTest extends AnyWordSpec with Matchers {
  "A unit quaternion" should {
    "convert to angle-axis" in {
      val unitQuaternion = UnitQuaternion(Quaternion(w = 0.5, x = 0.5, y = 0.5, z = 0.5))
      val rootThird = squareRoot(3).reciprocal
      unitQuaternion.axisAngle shouldEqual AxisAngle(Vector3(rootThird, rootThird, rootThird), 120.toRadians)
    }

    "convert to Euler angles" in {
      val unitQuaternion = UnitQuaternion(Quaternion(w = 0.5, x = 0.5, y = 0.5, z = 0.5))
      unitQuaternion.eulerAngles shouldEqual EulerAngles(90.toRadians, 0, 90.toRadians)
    }

    "convert to rotation matrix" in {
      val unitQuaternion = UnitQuaternion(Quaternion(w = 0.5, x = 0.5, y = 0.5, z = 0.5))
      unitQuaternion.rotationMatrix shouldEqual RotationMatrix33(Matrix33(Vector3.Y, Vector3.Z, Vector3.X))
    }

    "reversible with angle-axis" in {
      val unitQuaternion = exampleUnitQuaternion.next()
      unitQuaternion.axisAngle.unitQuaternion shouldEqual unitQuaternion
    }

    "reversible with rotation vector" in {
      val unitQuaternion = exampleUnitQuaternion.next()
      unitQuaternion.rotationVector.unitQuaternion shouldEqual unitQuaternion
    }

    "reversible with Euler angles" in {
      val unitQuaternion = exampleUnitQuaternion.next()

      if (!unitQuaternion.eulerAngles.isGimbalLocked)
        unitQuaternion.eulerAngles.unitQuaternion shouldEqual unitQuaternion
    }

    /**
      * Page 16 (156, 158)
      */
    "satisfy angular velocity to quaternion rates property" in {
      val q = exampleUnitQuaternion.next()
      val omega = exampleVector3.next()
      val p = Quaternion(0, omega.x, omega.y, omega.z)
      q.quaternionMatrix.matrix44 * p.vector4 shouldEqual q.ratesMatrix.transpose * omega
      q.conjugateQuaternionMatrix.matrix44 * p.vector4 shouldEqual q.conjugateRatesMatrix.transpose * omega
    }

    "satisfy component quaternion property" in {
      val eulerAngles = exampleEulerAngles.next()
      val roll = AxisAngle.normalize(Vector3.X, eulerAngles.ϕ).unitQuaternion
      val pitch = AxisAngle.normalize(Vector3.Y, eulerAngles.θ).unitQuaternion
      val yaw = AxisAngle.normalize(Vector3.Z, eulerAngles.ψ).unitQuaternion
      eulerAngles.unitQuaternion shouldEqual (roll ∘ pitch ∘ yaw)
    }

    /**
      * Page 15 (128)
      */
    "satisfy rotational product equivalence property" in {
      val q = exampleUnitQuaternion.next()
      val p = exampleUnitQuaternion.next()
      (q ∘ p).rotationMatrix shouldEqual (q.rotationMatrix * p.rotationMatrix)
    }

    "satisfy identity product identify" in {
      val p = exampleUnitQuaternion.next()
      p ∘ UnitQuaternion.zero shouldEqual p
      UnitQuaternion.zero ∘ p shouldEqual p
    }

    "deviate using small Euler Angle" in {
      val e = EulerAngles(0.0005, 0.001, 0.0015)
      val q = exampleUnitQuaternion.next()
      val p = e.limitUnitQuaternion ∘ q
      p shouldNot equal(q)
      p shouldEqual (e.unitQuaternion ∘ q)
    }
  }
}
