package com.peterpotts.trax.models.e

import com.peterpotts.trax.charting.Labels
import com.peterpotts.trax.charting.Suffices._
import com.peterpotts.trax.kalman._
import com.peterpotts.trax.math._
import com.peterpotts.trax.models._

/**
  * @author Peter Potts
  */
object ModelE extends Model {
  val title = "Model E"

  val stateLabels: Seq[String] = Labels(
    "d" -> V2,
    "v" -> V2,
    "a" -> V2,
    "theta" -> VZ,
    "omega" -> VZ,
    "alpha" -> VZ,
    "magOffB" -> V2,
    "gyrOff" -> VZ
  )

  val controlLabels: Seq[String] = Labels(
    "FB" -> V2,
    "tau" -> VZ
  )

  val measureLabels: Seq[String] =
    Labels(
      "dis" -> V2,
      "vel" -> V2,
      "accB" -> V2,
      "magB" -> V2,
      "opt" -> VZ,
      "gyr" -> VZ
    )

  val steps: Seq[Step] = {
    val dt = 1.0
    val mass = 10.0
    val inertia = 0.1
    val north = Vector2.Y
    val magOffB = Vector2.X
    val gyrOff = Vector1.identity

    val controller = ControllerE(mass, inertia, north, magOffB, gyrOff, dt)
    val predictor = PredictorE(mass, inertia, dt)
    val updater = UpdaterE(north)

    val start = {
      val target = controller.initial
      val actual = target.x
      val z = controller.measurements(actual, updater.v)
      val P = MatrixD.zero(XE.n, XE.n)
      val estimate = Estimate(actual, P)
      Step(0, actual, target.x, estimate, z, target.u)
    }

    val filter = ExtendedKalmanFilter(XE.n, UE.n, ZE.n, predictor, updater)

    def loop(step: Step): Step = {
      val actual = predictor.fuzzPredict(step.actual.x, step.u).f
      val target = controller.target(step.time, step.estimate.x, dt)
      val predict = filter.predict(step.u)(step.estimate)
      val z = controller.measurements(actual, updater.v)
      val update = filter.update(z)(predict)
      val u = controller.regulate(update.x, target)
      Step(step.time + dt, actual, target.x, update, z, u)
    }

    Iterator.iterate(start)(loop).take(601).toSeq
  }
}
