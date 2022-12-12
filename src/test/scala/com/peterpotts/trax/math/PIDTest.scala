package com.peterpotts.trax.math

import org.scalatest.matchers.should.Matchers
import org.scalatest.wordspec.AnyWordSpec

/**
  * @author Peter Potts
  */
class PIDTest extends AnyWordSpec with Matchers {
  "PID" should {
    "represent a simple mass-spring-damper system" in {
      val m = 1
      val b = 10
      val k = 20
      var x: Double = 0
      var v: Double = 0
      var a: Double = 0
      val δt: Double = 0.003
      val limiter = Limiter()
      val pid = PID(Kp = 34.68, Ki = 134.8, Kd = 2.23, limiter)

      // SP - Set point
      val sp: Double = 1

      // PV - Process variable
      def pv: Double = x

      def error: Double = sp - pv

      for (_ <- 0 to 1000) {
        // MV - Manipulated variable
        val mv = pid.control(error, δt)
        // Plant
        // m a + b v + k x = F
        val F = mv
        val nextA = (F - b * v - k * x) / m
        x = x + v * δt
        v = v + a * δt
        a = nextA
        //println(x)
      }
    }

    "xxx" in {
      val m = 1
      val b = 10
      val k = 20
      var x: Double = 0
      var v: Double = 0
      var a: Double = 0
      val δt: Double = 0.003
      val limiter = Limiter()
      val pid = PID(Kp = 34.68, Ki = 134.8, Kd = 2.23, limiter)

      // SP - Set point
      val sp: Double = 1

      // PV - Process variable
      def pv: Double = x

      def error: Double = sp - pv

      def autotuner(input: Double, i: Int): Double = {
        if ((i / 100) % 2 == 0) input + 10 else input - 10
      }

      for (i <- 0 to 1000) {
        val t = i * δt
        // MV - Manipulated variable
        val mv = autotuner(pid.control(error, δt), i)
        // Plant
        // m a + b v + k x = F
        val F = mv
        val nextA = (F - b * v - k * x) / m
        x = x + v * δt
        v = v + a * δt
        a = nextA
        println(x)
      }
    }
  }
}
