package com.peterpotts.trax.common

import org.scalatest.matchers.should.Matchers
import org.scalatest.wordspec.AnyWordSpec

/**
  * @author Peter Potts
  */
class TimeUSecTest extends AnyWordSpec with Matchers {
  "Unix microsecond time" should {
    "recognize the epoch" in {
      TimeUSec.toDateTime(0L).getMillis shouldEqual 0L
    }
  }
}
