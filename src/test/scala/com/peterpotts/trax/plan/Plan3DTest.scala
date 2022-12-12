package com.peterpotts.trax.plan

import com.peterpotts.trax.math.{DiagonalMatrix33, Vector3}
import org.scalatest.matchers.should.Matchers
import org.scalatest.wordspec.AnyWordSpec

/**
  * @author Peter Potts
  */
class Plan3DTest extends AnyWordSpec with Matchers {
  "Plan 3D" should {
    "calculate motion from force" in {
      val dt = 1.0
      val mass = 10.0
      val inertiaB = DiagonalMatrix33(Vector3.constant(0.1))
      val body0 = Body3D(mass = mass, inertiaB = inertiaB)
      val FB = Vector3.zero
      val plan3D: Plan3D = Force3D(FB)
      val body1 = plan3D.iterate(body0, dt)
      body0 shouldEqual body1
    }
  }
}
