package com.peterpotts.example

import com.peterpotts.example.TrivialExample._
import org.scalatest.matchers.should.Matchers
import org.scalatest.wordspec.AnyWordSpec

import scala.util.Success

/**
  * @author Peter Potts
  */
class ExampleCompositeTest extends AnyWordSpec with Matchers {
  "A trivial example either" should {
    "be trivial" in {
      exampleEither(exampleLong, exampleString).next() shouldEqual Right("0" * defaultSize)
    }
  }

  "A trivial example option" should {
    "be trivial" in {
      exampleOption(exampleInt).next() shouldEqual Some(0)
    }
  }

  "A trivial example try" should {
    "be trivial" in {
      exampleTry(exampleBoolean).next() shouldEqual Success(false)
    }
  }

  "A trivial example future" should {
    "be trivial" in {
      exampleFuture(exampleDouble).next().value shouldEqual Some(Success(0.0))
    }
  }

  "A trivial example trampoline" should {
    "be trivial" in {
      val example = Example[Boolean](_ => throw new RuntimeException)
      val trampoline = exampleTrampoline(exampleList(example)).next()
      intercept[RuntimeException](trampoline.run)
    }
  }
}
