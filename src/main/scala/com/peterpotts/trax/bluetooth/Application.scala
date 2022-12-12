package com.peterpotts.trax.bluetooth

import java.io._

import com.peterpotts.trax.common.LoanPattern._

import scala.concurrent.TimeoutException

/**
  * @author Peter Potts
  */
object Application {
  private val interval = 300
  private val device = "/dev/tty.HC-06-DevB-1"

  def main(args: Array[String]): Unit = {
    val file = new File(device)

    using(new BufferedReader(new FileReader(file)))(_.close()) { input =>
      using(new BufferedWriter(new FileWriter(file)))(_.close()) { output =>
        while (true) {
          val start = System.currentTimeMillis()
          output.write("get\n")
          output.flush()
          val line = input.readLine()
          val call = System.currentTimeMillis()
          val Array(x, y, z) = line.split(',')
          println(s"${call - start}, $x, $y, $z")
          val end = System.currentTimeMillis()
          val wait = interval - (end - start)
          if (wait < 0) throw new TimeoutException
          if (wait > 0) Thread.sleep(wait)
        }
      }
    }
  }

  def main2(args: Array[String]): Unit = {
    val array = new Array[(Int, Int, Int)](200)
    val file = new File(device)

    using(new BufferedReader(new FileReader(file)))(_.close()) { input =>
      using(new BufferedWriter(new FileWriter(file)))(_.close()) { output =>
        for (index <- array.indices) {
          val start = System.currentTimeMillis()
          output.write("get\n")
          output.flush()
          val line = input.readLine()
          val Array(x, y, z) = line.split(',')
          array(index) = (x.toInt, y.toInt, z.toInt)
          val end = System.currentTimeMillis()
          val wait = interval - (end - start)
          if (wait < 0) throw new TimeoutException
          if (wait > 0) Thread.sleep(wait)
        }
      }
    }

    for (index <- array.indices) {
      // Standard deviation 115 or 153 or 142
      println(s"${array(index)._3}")
    }
  }
}
