package com.peterpotts.trax.math

import com.peterpotts.example.RandomExample._
import com.peterpotts.trax.math.DoubleEquality._
import com.peterpotts.trax.math.GeometricEquality._
import com.peterpotts.trax.math.GeometryExamples._
import org.scalatest.matchers.should.Matchers
import org.scalatest.wordspec.AnyWordSpec

/**
  * @author Peter Potts
  */
class QuaternionTest extends AnyWordSpec with Matchers {
  "A quaternion" should {
    "satisfy identity product property" in {
      val q = exampleQuaternion.next()
      (q ∘ Quaternion.identity) shouldEqual q
      (Quaternion.identity ∘ q) shouldEqual q
    }

    "satisfy real commute product property" in {
      val q = exampleQuaternion.next()
      val p = exampleQuaternion.next().scalar
      (q ∘ p) shouldEqual (p ∘ q)
    }

    "satisfy conjugate product property" in {
      val q = exampleQuaternion.next()
      val p = exampleQuaternion.next()
      (q ∘ p).conjugate shouldEqual (p.conjugate ∘ q.conjugate)
    }

    "satisfy norm product property" in {
      val q = exampleQuaternion.next()
      val p = exampleQuaternion.next()
      (q ∘ p).norm shouldEqual (q.norm * p.norm)
    }

    "satisfy conjugate involution property" in {
      val quaternion = exampleQuaternion.next()
      quaternion.conjugate.conjugate shouldEqual quaternion
    }

    "satisfy inverse property" in {
      val quaternion = exampleQuaternion.next()

      if (quaternion !== Quaternion.zero) {
        (quaternion.inverse ∘ quaternion) shouldEqual Quaternion.identity
        (quaternion ∘ quaternion.inverse) shouldEqual Quaternion.identity
        quaternion.inverse.inverse shouldEqual quaternion
      }
    }

    /**
      * Page 14 (103)
      */
    "satisfy definition of hamiltonian product" in {
      val q = exampleQuaternion.next()
      val p = exampleQuaternion.next()
      val qp = q ∘ p
      qp.w shouldEqual q.w * p.w - q.vector3.dotProduct(p.vector3)
      qp.vector3 shouldEqual q.vector3 * p.w + p.vector3 * q.w - q.vector3.skewSymmetric * p.vector3
    }

    /**
      * Page 14 (106, 107)
      */
    "satisfy quaternion matrix product property" in {
      val q = exampleQuaternion.next()
      val p = exampleQuaternion.next()
      (q ∘ p) shouldEqual (q.quaternionMatrix * p)
      (q ∘ p) shouldEqual (p.conjugateQuaternionMatrix * q)
      (p ∘ q) shouldEqual (p.quaternionMatrix * q)
      (p ∘ q) shouldEqual (q.conjugateQuaternionMatrix * p)
    }

    /**
      * Page 14 (112, 113)
      */
    "satisfy conjugate transpose property" in {
      val quaternion = exampleQuaternion.next()
      quaternion.conjugate.quaternionMatrix shouldEqual quaternion.quaternionMatrix.transpose
      quaternion.conjugate.conjugateQuaternionMatrix shouldEqual quaternion.conjugateQuaternionMatrix.transpose
    }

    "satisfy inverse commutative property" in {
      val q = exampleQuaternion.next()
      val p = exampleQuaternion.next()
      (q.inverse ∘ p).inverse shouldEqual (p.inverse ∘ q)
    }
  }
}
