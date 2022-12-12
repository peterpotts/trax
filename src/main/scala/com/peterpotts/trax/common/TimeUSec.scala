package com.peterpotts.trax.common

import org.joda.time.{DateTime, DateTimeZone}

/**
  * @author Peter Potts
  */
object TimeUSec {
  def now(): Long = System.currentTimeMillis() * 1000L

  def toDateTime(value: Long): DateTime = new DateTime(value / 1000L, DateTimeZone.getDefault)
}
