package com.peterpotts.example

import com.peterpotts.example.TrivialExample._
import org.scalatest.matchers.should.Matchers
import org.scalatest.wordspec.AnyWordSpec

/**
  * @author Peter Potts
  */
class ExampleCollectionTest extends AnyWordSpec with Matchers {
  "A trivial example pick" should {
    "be trivial" in {
      examplePick(3 to 5).next() shouldEqual 3
    }
  }

  "A trivial example shuffle" should {
    "be trivial" in {
      exampleShuffle(exampleInt, exampleInt).next() shouldEqual 0
    }
  }

  "A trivial example stream" should {
    "be trivial" in {
      exampleStream(exampleBoolean).next().head shouldEqual false
    }
  }

  "A trivial example list" should {
    "be trivial" in {
      exampleList(exampleLong).next() shouldEqual List.fill(defaultSize)(0L)
    }

    "not blow the stack" in {
      exampleList(exampleLong, 10000).next()
    }
  }

  "A trivial example array" should {
    "be trivial" in {
      exampleArray(exampleInt).next() shouldEqual Array.fill(defaultSize)(0)
    }
  }

  "A trivial example vector" should {
    "be trivial" in {
      exampleVector(exampleString).next() shouldEqual Vector.fill(defaultSize)("0" * defaultSize)
    }
  }

  "A trivial example map" should {
    "be trivial" in {
      exampleMap(exampleInt, exampleBoolean, size = 1).next() shouldEqual Map(0 -> false)
    }
  }

  "A trivial example bytes" should {
    "be trivial" in {
      exampleBytes.next() shouldEqual Array.fill(defaultSize)(0.toByte)
    }
  }
}
