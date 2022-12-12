package com.peterpotts.trax.math

/**
  * A proportional–integral–derivative controller.
  *
  * @author Peter Potts
  * @param Kp      Proportional band / gain
  * @param Ki      Integral gain / reset
  * @param Kd      Derivative gain / rate
  * @param limiter Integral windup limiter
  *
  * @author Peter Potts
  */
case class PID(Kp: Double, Ki: Double, Kd: Double, limiter: Limiter) {
  var p: Double = 0
  var i: Double = 0
  var d: Double = 0

  /**
    * PID algorithm.
    *
    * @param error Set point [SP] minus process variable [PV]
    * @param δt    Delta time
    * @return Manipulated variable [MV] or control variable [CV]
    */
  def control(error: Double, δt: Double): Double = {
    i = limiter(i + error * δt)
    d = (error - p) / δt
    p = error
    Kp * p + Ki * i + Kd * d
  }
}

object PID {
  /**
    * Ziegler–Nichols heuristic tuning method.
    *
    * @param Kp      Ultimate gain
    * @param Tu      Oscillation period
    * @param limiter Integral windup limiter
    */
  def apply(Kp: Double, Tu: Double, limiter: Limiter): PID =
    PID(Kp = Kp, Ki = 2.0 * Kp / Tu, Kd = Kp * Tu / 8.0, limiter)
}
