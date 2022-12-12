package com.peterpotts.trax.models.c

import com.peterpotts.trax.charting.Labels
import com.peterpotts.trax.charting.Suffices._
import com.peterpotts.trax.kalman._
import com.peterpotts.trax.math._
import com.peterpotts.trax.models._

/**
  * @author Peter Potts
  */
object ModelC2 extends Model {
  val title = "Model C 2"
  val stateLabels: Seq[String] = Labels("d" -> V2, "v" -> V2)
  val controlLabels: Seq[String] = Labels("F" -> V2)
  val measureLabels: Seq[String] = Labels("dis" -> V2)

  val steps: Seq[Step] = {
    val dt = 1.0
    val mass = 100.0
    val inertia = 0.1

    val controller = ControllerC2(mass, inertia, dt)
    val predictor = PredictorC(mass, dt)
    val updater = UpdaterC

    val start = {
      val target = controller.initial
      val actual = target.x
      val z = controller.measurements(actual, updater.v)
      val P = MatrixD.zero(XC.n, XC.n)
      val estimate = Estimate(actual, P)
      Step(0, actual, target.x, estimate, z, target.u)
    }

    val filter = ExtendedKalmanFilter(XC.n, UC.n, ZC.n, predictor, updater)

    def loop(step: Step): Step = {
      val actual = predictor.fuzzPredict(step.actual.x, step.u).f
      val target = controller.target(step.time, step.estimate.x, dt)
      val predict = filter.predict(step.u)(step.estimate)
      val z = controller.measurements(actual, updater.v)
      val update = filter.update(z)(predict)
      val u = controller.regulate(update.x, target)
      Step(step.time + dt, actual, target.x, update, z, u)
    }

    Iterator.iterate(start)(loop).take(401).toSeq
  }
}
