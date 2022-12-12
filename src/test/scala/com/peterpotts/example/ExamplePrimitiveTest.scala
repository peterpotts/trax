package com.peterpotts.example

import com.peterpotts.example.TrivialExample._
import org.scalatest.matchers.should.Matchers
import org.scalatest.wordspec.AnyWordSpec

/**
  * @author Peter Potts
  */
class ExamplePrimitiveTest extends AnyWordSpec with Matchers {
  "A trivial example long" should {
    "be trivial" in {
      exampleLong.next() shouldEqual 0L
    }
  }

  "A trivial example int" should {
    "be trivial" in {
      exampleInt.next() shouldEqual 0
    }
  }

  "A trivial example boolean" should {
    "be trivial" in {
      exampleBoolean.next() shouldEqual false
    }
  }

  "A trivial example byte" should {
    "be trivial" in {
      exampleByte.next() shouldEqual 0.toByte
    }
  }

  "A trivial example double" should {
    "be trivial" in {
      exampleDouble.next() shouldEqual 0.0
    }
  }

  "A trivial example unit" should {
    "be trivial" in {
      exampleUnit.next() shouldEqual {}
    }
  }

  "A trivial example lift" should {
    "lift" in {
      exampleLift(0).next() shouldEqual 0
    }
  }
}