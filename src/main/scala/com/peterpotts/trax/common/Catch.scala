package com.peterpotts.trax.common

import scala.util.control.NonFatal

/**
  * @author Peter Potts
  */
object Catch {
  def apply[T](block: => T): T = try block catch {
    case NonFatal(exception) =>
      exception.printStackTrace()
      throw exception
  }
}
