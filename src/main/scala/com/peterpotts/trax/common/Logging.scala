package com.peterpotts.trax.common

import org.slf4j.{Logger, LoggerFactory}

/**
  * @author Peter Potts
  */
trait Logging {
  @transient protected lazy val logger: Logger = LoggerFactory.getLogger(getClass.getName)
}
