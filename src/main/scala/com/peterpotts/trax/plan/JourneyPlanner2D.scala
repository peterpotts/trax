package com.peterpotts.trax.plan

import com.peterpotts.trax.kalman.X
import com.peterpotts.trax.math._

/**
  * @author Peter Potts
  */
case class JourneyPlanner2D(initial: Body2D, legs: List[Leg2D], dt: Double) extends Planner2D {
  val plans: LazyList[Body2D] = loop(initial, dt, legs)

  def plan(time: Double, x: X, dt: Double): Body2D = plans((time / dt).round.toInt)

  private def loop(body: Body2D, time: Double, legs: List[Leg2D]): LazyList[Body2D] =
    legs match {
      case Nil => loop(body, time, this.legs)
      case head :: tail =>
        if (time <= head.time) {
          val nextBody = head.plan(body, dt)
          nextBody #:: loop(nextBody, time + dt, legs)
        } else {
          loop(body, time - head.time, tail)
        }
    }
}

object JourneyPlanner2D {
  def example0(mass: Double, inertia: Double, dt: Double): Planner2D = {
    val circularPlanner = CircularPlanner2D(mass, inertia)
    val body = circularPlanner.initial
    import circularPlanner.{force, period}
    val legs = List(Leg2D(period, Force2D(Vector2(0, force))))
    JourneyPlanner2D(body, legs, dt)
  }

  def example1(mass: Double, inertia: Double, dt: Double): Planner2D = {
    val circularPlanner = CircularPlanner2D(mass, inertia)
    val body = Body2D(mass = mass, inertia = inertia)
    import circularPlanner.{angularVelocity, force, period}
    val tau = inertia * angularVelocity / period

    val legs = List(
      Leg2D(period, Torque2D(Vector1(tau))),
      Leg2D(period, Force2D(Vector2.Y * force)),
      Leg2D(period, Cruise2D),
      Leg2D(period, Force2D(Vector2.Y * -force)),
      Leg2D(period, Cruise2D),
      Leg2D(period, Torque2D(Vector1(-tau)))
    )

    JourneyPlanner2D(body, legs, dt)
  }

  def example3(mass: Double, inertia: Double, dt: Double): Planner2D = {
    val body = Body2D(mass = mass, inertia = inertia)

    val legs = List(
      Leg2D(10, Force2D(Vector2.X)),
      Leg2D(10, Cruise2D),
      Leg2D(10, Force2D(Vector2.X.negate)),
      Leg2D(10, Cruise2D),
    )

    JourneyPlanner2D(body, legs, dt)
  }
}
