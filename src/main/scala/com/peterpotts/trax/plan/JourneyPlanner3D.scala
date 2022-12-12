package com.peterpotts.trax.plan

import com.peterpotts.trax.kalman.X
import com.peterpotts.trax.math._

/**
  * @author Peter Potts
  */
case class JourneyPlanner3D(initial: Body3D, legs: List[Leg3D], dt: Double) extends Planner3D {
  val plans: LazyList[Body3D] = loop(initial, dt, legs)

  def plan(time: Double, x: X, dt: Double): Body3D = plans((time / dt).round.toInt)

  private def loop(body: Body3D, time: Double, legs: List[Leg3D]): LazyList[Body3D] =
    legs match {
      case Nil => loop(body, time, this.legs)
      case head :: tail =>
        if (time <= head.time) {
          val nextBody = head.plan.iterate(body, dt)
          nextBody #:: loop(nextBody, time + dt, legs)
        } else {
          loop(body, time - head.time, tail)
        }
    }
}

object JourneyPlanner3D {
  def example0(mass: Double, inertiaB: DiagonalMatrix33, dt: Double): Planner3D = {
    val circularPlanner = CircularPlanner3D(mass, inertiaB)
    val body = circularPlanner.initial
    import circularPlanner.{force, period}
    val legs = List(Leg3D(period, Force3D(Vector3.Y * force)))
    JourneyPlanner3D(body, legs, dt)
  }

  def example1(mass: Double, inertiaB: DiagonalMatrix33, dt: Double): Planner3D = {
    val circularPlanner = CircularPlanner3D(mass, inertiaB)
    val body = Body3D(mass = mass, inertiaB = inertiaB)
    import circularPlanner.{angularVelocity, force, period}
    val tauB = inertiaB.vector3.z * angularVelocity / period

    val legs =
      List(
        Leg3D(period, Torque3D(Vector3.Z * tauB)),
        Leg3D(period, Force3D(Vector3.Y * force)),
        Leg3D(period, Cruise3D),
        Leg3D(period, Force3D(Vector3.Y * -force)),
        Leg3D(period, Cruise3D),
        Leg3D(period, Torque3D(Vector3.Z * -tauB))
      )

    JourneyPlanner3D(body, legs, dt)
  }
}
