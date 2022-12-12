package com.peterpotts.trax.neuron

/**
  * @author Peter Potts
  */
object Spring {
  def main(args: Array[String]): Unit = {
    val m = 1.0
    val c = 8.8
    val k = 40.0

    var Kc = 10.0
    var Ti = 999.0
    var Td = 0.0

    val dt = 0.001

    val N = 10000

    val x = Array.fill(N)(0.0)
    val v = Array.fill(N)(0.0)
    val a = Array.fill(N)(0.0)
    val e = Array.fill(N)(0.0)
    val pp = Array.fill(N)(0.0)
    val ii = Array.fill(N)(0.0)
    val dd = Array.fill(N)(0.0)
    val mv = Array.fill(N)(0.0)

    val sp = 1.0

    def pv: Array[Double] = x

    x(0) = 0.0
    v(0) = 0.0
    a(0) = 0.0
    e(0) = sp - pv(0)
    pp(0) = e(0)
    ii(0) = 0.0
    dd(0) = 0.0
    mv(0) = Kc * (pp(0) + ii(0) / Ti + dd(0) * Td)

    val one = Array.fill(3)(Neuron(Vector.fill(7)(0.5)))
    val two = Array.fill(3)(Neuron(Vector.fill(3)(0.5)))
    val nu = 1.0

    for (n <- 1 until 10000) {
      x(n) = x(n - 1) + v(n - 1) * dt
      v(n) = v(n - 1) + a(n - 1) * dt
      a(n) = (mv(n - 1) - c * v(n - 1) - k * x(n - 1)) / m
      e(n) = sp - pv(n)
      pp(n) = e(n)
      ii(n) = ii(n - 1) + e(n) * dt
      dd(n) = (e(n) - pp(n - 1)) / dt
      mv(n) = Kc * (pp(n) + ii(n) / Ti + dd(n) * Td)

      val input = Vector(sp, sp, pv(n), pv(n - 1), mv(n), mv(n - 1), e(n))
      val hidden: Vector[Double] = one.toVector.map(_.output(input))
      val output: Vector[Double] = two.toVector.map(_.output(hidden))

      for (j <- 0 until 3) {
        val weights = two(j).weights.zipWithIndex.map {
          case (weight, i) =>
            weight - nu * e(n) * output(j) * (1.0 - output(j)) * hidden(i)
        }

        two(j) = Neuron(weights)
      }

        for (j <- 0 until 3) {
        val weights = one(j).weights.zipWithIndex.map {
          case (weight, i) =>
            val back: Double = one(0).weights(i)
            weight - nu * back * hidden(j) * (1.0 - hidden(j)) * input(i)
        }

        one(j) = Neuron(weights)
      }

      val Kp = output(0)
      val Ki = output(1)
      val Kd = output(2)

      Kc = Kp
      Ti = Kp / Ki
      Td = Kd / Kp

      println(pv(n))
      println(s"Kp=$Kp")
    }
  }
}
