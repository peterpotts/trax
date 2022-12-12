package com.peterpotts.trax.math

import com.peterpotts.example.RandomExample._
import com.peterpotts.trax.math.GeometryExamples._
import org.scalatest.matchers.should.Matchers
import org.scalatest.wordspec.AnyWordSpec

class NormalizeTest extends AnyWordSpec with Matchers {
  "Normalize" should {
    "normalize Euler Angles" in {
      exampleList(exampleEulerAngles, 1000).next().foreach { e =>
        val q = e.unitQuaternion
        val a = q.axisAngle
        val r = a.rotationVector
        val a2 = r.axisAngle
        val q2 = a2.unitQuaternion
        val e2 = q2.eulerAngles

        e2 === e shouldEqual true
        q2 === q shouldEqual true
        a2 === a shouldEqual true
      }
    }

    "normalize unit quaternion" in {
      exampleList(exampleUnitQuaternion, 1000).next().foreach { q =>
        val e = q.eulerAngles
        val q2 = e.unitQuaternion

        val a = q.axisAngle
        val r = a.rotationVector
        val a2 = r.axisAngle
        val q3 = a2.unitQuaternion

        q2 === q shouldEqual true
        a2 === a shouldEqual true
        q3 === q shouldEqual true
      }
    }

    "normalize axis angle" in {
      exampleList(exampleAxisAngle, 1000).next().foreach { a =>
        val q = a.unitQuaternion
        val e = q.eulerAngles
        val q2 = e.unitQuaternion
        val a2 = q2.axisAngle

        val r = a.rotationVector
        val a3 = r.axisAngle

        q2 === q shouldEqual true
        a2 === a shouldEqual true
        a3 === a shouldEqual true
      }
    }

    "normalize rotation vector" in {
      exampleList(exampleRotationVector, 1000).next().foreach { r =>
        val a = r.axisAngle
        val q = a.unitQuaternion
        val e = q.eulerAngles
        val q2 = e.unitQuaternion
        val a2 = q2.axisAngle
        val r2 = a2.rotationVector

        q2 === q shouldEqual true
        a2 === a shouldEqual true
        r2 === r shouldEqual true
      }
    }
  }
}
