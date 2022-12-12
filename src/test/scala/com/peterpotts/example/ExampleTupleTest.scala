package com.peterpotts.example

import com.peterpotts.example.TrivialExample._
import org.scalatest.matchers.should.Matchers
import org.scalatest.wordspec.AnyWordSpec

/**
  * @author Peter Potts
  */
class ExampleTupleTest extends AnyWordSpec with Matchers {
  "A trivial example tuple 2" should {
    "be trivial" in {
      exampleTuple2(exampleLong, exampleString).next() shouldEqual Tuple2(0L, "0" * defaultSize)
    }
  }

  "A trivial example tuple 3" should {
    "be trivial" in {
      exampleTuple3(exampleLong, exampleString, exampleBoolean).next() shouldEqual Tuple3(0L, "0" * defaultSize, false)
    }
  }

  "A trivial example tuple 4" should {
    "be trivial" in {
      exampleTuple4(exampleLong, exampleString, exampleBoolean, exampleInt).next() shouldEqual
        Tuple4(0L, "0" * defaultSize, false, 0)
    }
  }

  "A trivial example tuple 5" should {
    "be trivial" in {
      exampleTuple5(exampleLong, exampleString, exampleBoolean, exampleInt, exampleByte).next() shouldEqual
        Tuple5(0L, "0" * defaultSize, false, 0, 0.toByte)
    }
  }
}
