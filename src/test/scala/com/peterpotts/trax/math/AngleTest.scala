package com.peterpotts.trax.math

import com.peterpotts.example.RandomExample._
import com.peterpotts.trax.math.DoubleEquality._
import com.peterpotts.trax.math.GeometryExamples._
import org.scalatest.matchers.should.Matchers
import org.scalatest.wordspec.AnyWordSpec

import scala.math._

class AngleTest extends AnyWordSpec with Matchers {
  "Angle" should {
    "normalize" in {
      exampleStream(examplePlusMinusFourPi).next().take(1000).foreach(Angle.normalize)
    }

    "normalize to interval [-π, π)" in {
      exampleStream(examplePlusMinusFourPi).next().take(1000).foreach { x =>
        Angle.normalizePlusMinus(x) shouldBe >=(MinusPi)
        Angle.normalizePlusMinus(x) shouldBe <(Pi)
      }

      Angle.normalizePlusMinus(MinusTwoPi) shouldEqual 0.0
      Angle.normalizePlusMinus(MinusOneHalfPi) shouldEqual HalfPi
      Angle.normalizePlusMinus(MinusPi) shouldEqual MinusPi
      Angle.normalizePlusMinus(MinusHalfPi) shouldEqual MinusHalfPi
      Angle.normalizePlusMinus(0.0) shouldEqual 0.0
      Angle.normalizePlusMinus(HalfPi) shouldEqual HalfPi
      Angle.normalizePlusMinus(Pi) shouldEqual MinusPi
      Angle.normalizePlusMinus(OneHalfPi) shouldEqual MinusHalfPi
      Angle.normalizePlusMinus(TwoPi) shouldEqual 0.0

      Angle.normalizePlusMinus(10.0.toRadians) shouldEqual 10.0.toRadians
      Angle.normalizePlusMinus(100.0.toRadians) shouldEqual 100.0.toRadians
      Angle.normalizePlusMinus(190.0.toRadians) shouldEqual -170.0.toRadians
      Angle.normalizePlusMinus(370.0.toRadians) shouldEqual 10.0.toRadians
      Angle.normalizePlusMinus(730.0.toRadians) shouldEqual 10.0.toRadians
      Angle.normalizePlusMinus(-10.0.toRadians) shouldEqual -10.0.toRadians
      Angle.normalizePlusMinus(-100.0.toRadians) shouldEqual -100.0.toRadians
      Angle.normalizePlusMinus(-190.0.toRadians) shouldEqual 170.0.toRadians
      Angle.normalizePlusMinus(-370.0.toRadians) shouldEqual -10.0.toRadians
      Angle.normalizePlusMinus(-730.0.toRadians) shouldEqual -10.0.toRadians
    }

    "normalize to interval [0, 2π)" in {
      exampleStream(examplePlusMinusFourPi).next().take(1000000).foreach { x =>
        Angle.normalizePlus(x) shouldBe >=(0.0)
        Angle.normalizePlus(x) shouldBe <=(TwoPi)
      }

      Angle.normalizePlus(MinusTwoPi) shouldEqual 0.0
      Angle.normalizePlus(MinusOneHalfPi) shouldEqual HalfPi
      Angle.normalizePlus(MinusPi) shouldEqual Pi
      Angle.normalizePlus(MinusHalfPi) shouldEqual OneHalfPi
      Angle.normalizePlus(0.0) shouldEqual 0.0
      Angle.normalizePlus(HalfPi) shouldEqual HalfPi
      Angle.normalizePlus(Pi) shouldEqual Pi
      Angle.normalizePlus(OneHalfPi) shouldEqual OneHalfPi
      Angle.normalizePlus(TwoPi) shouldEqual 0.0

      Angle.normalizePlus(10.0.toRadians) shouldEqual 10.0.toRadians
      Angle.normalizePlus(100.0.toRadians) shouldEqual 100.0.toRadians
      Angle.normalizePlus(190.0.toRadians) shouldEqual 190.0.toRadians
      Angle.normalizePlus(370.0.toRadians) shouldEqual 10.0.toRadians
      Angle.normalizePlus(730.0.toRadians) shouldEqual 10.0.toRadians
      Angle.normalizePlus(-10.0.toRadians) shouldEqual 350.0.toRadians
      Angle.normalizePlus(-100.0.toRadians) shouldEqual 260.0.toRadians
      Angle.normalizePlus(-190.0.toRadians) shouldEqual 170.0.toRadians
      Angle.normalizePlus(-370.0.toRadians) shouldEqual 350.0.toRadians
      Angle.normalizePlus(-730.0.toRadians) shouldEqual 350.0.toRadians
    }
  }
}
