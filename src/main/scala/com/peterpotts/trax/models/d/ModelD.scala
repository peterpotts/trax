package com.peterpotts.trax.models.d

import com.peterpotts.trax.charting.Labels
import com.peterpotts.trax.charting.Suffices._
import com.peterpotts.trax.kalman._
import com.peterpotts.trax.math._
import com.peterpotts.trax.models._

/**
  * @author Peter Potts
  */
object ModelD extends Model {
  val title = "Model D"
  val stateLabels: Seq[String] = Labels("d" -> V2, "v" -> V2, "a" -> V2, "theta" -> VZ, "omega" -> VZ, "alpha" -> VZ)
  val controlLabels: Seq[String] = Labels("FB" -> V2, "tau" -> VZ)
  val measureLabels: Seq[String] = Labels("dis" -> V2, "vel" -> V2, "accB" -> V2, "magB" -> V2, "gyr" -> VZ)

  val steps: Seq[Step] = {
    val dt = 1.0
    val mass = 10.0
    val inertia = 0.1
    val north = Vector2.Y

    val controller = ControllerD(mass, inertia, north, dt)
    val predictor = PredictorD(mass, inertia, dt)
    val updater = UpdaterD(north)

    val start = {
      val target = controller.initial
      val actual = target.x
      val z = controller.measurements(actual, updater.v)
      val P = MatrixD.zero(XD.n, XD.n)
      val estimate = Estimate(actual, P)
      Step(0, actual, target.x, estimate, z, target.u)
    }

    val filter = ExtendedKalmanFilter(XD.n, UD.n, ZD.n, predictor, updater)

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
