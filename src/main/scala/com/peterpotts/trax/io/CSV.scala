package com.peterpotts.trax.io

import java.io.{File, PrintWriter}

/**
  * @author Peter Potts
  */
object CSV {
  def apply(file: File)(writer: (Seq[Any] => Unit) => Unit): Unit = {
    val printWriter = new PrintWriter(file)
    try writer(line => printWriter.println(line.mkString(","))) finally printWriter.close()
  }
}
