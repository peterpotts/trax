package com.peterpotts.trax.mavlink.io

import com.peterpotts.trax.common.Logging

/**
  * @author Peter Potts
  */
abstract class PromptLogging(val prompt: String) extends Logging {
  def prompt(message: String): Unit = logger.info(s"$prompt> $message")
}
