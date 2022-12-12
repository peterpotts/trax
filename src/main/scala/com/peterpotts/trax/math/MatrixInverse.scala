package com.peterpotts.trax.math

import scala.math.abs

/**
  * @author Peter Potts
  */
object MatrixInverse {
  def apply(matrix: MatrixD): MatrixD = {
    val a = matrix.map(_.toArray).toArray
    val n = a.length
    val x = Array.fill(n)(Array.fill(n)(0.0))
    val b = Array.fill(n)(Array.fill(n)(0.0))
    val index = Array.fill(n)(0)
    for (i <- 0 until n) b(i)(i) = 1.0

    gaussian(a, index)

    for (i <- 0 until n - 1)
      for (j <- i + 1 until n)
        for (k <- 0 until n)
          b(index(j))(k) = b(index(j))(k) - a(index(j))(i) * b(index(i))(k)

    for (i <- 0 until n) {
      x(n - 1)(i) = b(index(n - 1))(i) / a(index(n - 1))(n - 1)

      for (j <- n - 2 to 0 by -1) {
        x(j)(i) = b(index(j))(i)

        for (k <- j + 1 until n)
          x(j)(i) = x(j)(i) - a(index(j))(k) * x(k)(i)

        x(j)(i) = x(j)(i) / a(index(j))(j)
      }
    }

    x.map(_.toVector).toVector
  }

  private def gaussian(a: Array[Array[Double]], index: Array[Int]): Unit = {
    val n = index.length
    val c = Array.fill(n)(0.0)

    for (i <- 0 until n) index(i) = i

    for (i <- 0 until n) {
      var c1 = 0.0

      for (j <- 0 until n) {
        val c0 = abs(a(i)(j))
        if (c0 > c1) c1 = c0
      }

      c(i) = c1
    }

    var k = 0

    for (j <- 0 until n - 1) {
      var pi1 = 0.0

      for (i <- j until n) {
        val pi0 = abs(a(index(i))(j)) / c(index(i))

        if (pi0 > pi1) {
          pi1 = pi0
          k = i
        }
      }

      val tmp = index(j)
      index(j) = index(k)
      index(k) = tmp

      for (i <- j + 1 until n) {
        val pj = a(index(i))(j) / a(index(j))(j)
        a(index(i))(j) = pj

        for (l <- j + 1 until n) a(index(i))(l) = a(index(i))(l) - pj * a(index(j))(l)
      }
    }
  }
}
