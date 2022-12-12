package com.peterpotts.example

import java.net.InetAddress
import java.util.UUID

import com.peterpotts.example.TrivialExample._
import org.scalatest.matchers.should.Matchers
import org.scalatest.wordspec.AnyWordSpec

import scala.concurrent.duration._

/**
  * @author Peter Potts
  */
class ExampleSpecialTest extends AnyWordSpec with Matchers {
  "A trivial example big int" should {
    "be trivial" in {
      exampleBigInt.next() shouldEqual BigInt(0)
    }
  }

  "A trivial example big decimal" should {
    "be trivial" in {
      exampleBigDecimal.next() shouldEqual BigDecimal(0.0)
    }
  }

  "A trivial example alphanumeric" should {
    "be trivial" in {
      exampleAlphanumeric.next() shouldEqual '0'
    }
  }

  "A trivial example string" should {
    "be trivial" in {
      exampleString.next() shouldEqual "0" * defaultSize
    }
  }

  "A trivial example port" should {
    "be trivial" in {
      examplePort.next() shouldEqual 49152
    }
  }

  "A trivial example time unit" should {
    "be trivial" in {
      exampleTimeUnit.next() shouldEqual DAYS
    }
  }

  "A trivial example finite duration" should {
    "be trivial" in {
      exampleFiniteDuration.next() shouldEqual 1.day
    }
  }

  "A trivial example UUID" should {
    "be trivial" in {
      exampleUUID.next() shouldEqual UUID.nameUUIDFromBytes(Array.fill(16)(0.toByte))
    }
  }

  "A trivial example INET address" should {
    "be trivial" in {
      exampleInetAddress.next() shouldEqual InetAddress.getByName("0.0.0.0")
    }
  }
}