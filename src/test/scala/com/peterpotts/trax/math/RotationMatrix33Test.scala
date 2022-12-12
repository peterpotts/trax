package com.peterpotts.trax.math

import com.peterpotts.trax.math.DoubleEquality._
import com.peterpotts.trax.math.GeometricEquality._
import com.peterpotts.trax.math.GeometryExamples._
import org.scalatest.matchers.should.Matchers
import org.scalatest.wordspec.AnyWordSpec

/**
  * @author Peter Potts
  */
class RotationMatrix33Test extends AnyWordSpec with Matchers {
  "Rotation matrix" should {
    "convert roll to Euler Angle" in {
      val roll = exampleAngle.next()
      RotationMatrix33.X(roll).eulerAngles shouldEqual EulerAngles(roll, 0, 0)
    }

    "convert pitch to Euler Angle" in {
      val pitch = exampleAngle.next()
      RotationMatrix33.Y(pitch).eulerAngles shouldEqual EulerAngles(0, pitch, 0)
    }

    "convert yaw to Euler Angle" in {
      val yaw = exampleAngle.next()
      RotationMatrix33.Z(yaw).eulerAngles shouldEqual EulerAngles(0, 0, yaw)
    }

    "reversible with Euler angles" in {
      val rotationMatrix = exampleRotationMatrix.next()
      rotationMatrix.eulerAngles.rotationMatrix shouldEqual rotationMatrix
    }

    "satisfy unit determinant property" in {
      val rotationMatrix = exampleRotationMatrix.next()
      rotationMatrix.matrix33.determinant shouldEqual 1.0
    }

    "satisfy inverse is transpose property" in {
      val rotationMatrix = exampleRotationMatrix.next()
      rotationMatrix.matrix33.inverse shouldEqual rotationMatrix.matrix33.transpose
    }

    "satisfy inverse rotation bracket property" in {
      val rotationMatrix = exampleRotationMatrix.next()
      val matrix33 = exampleMatrix33.next()
      val a = rotationMatrix.inverse.matrix33 * matrix33 * rotationMatrix.matrix33
      val b = rotationMatrix.inverse.matrix33 * matrix33.inverse * rotationMatrix.matrix33
      a.inverse shouldEqual b
    }

    "satisfy transitive property with cross product" in {
      val a = exampleRotationMatrix.next().matrix33
      val b = exampleVector3.next()
      val c = exampleVector3.next()
      a * (b crossProduct c) shouldEqual ((a * b) crossProduct (a * c))
    }
  }
}
