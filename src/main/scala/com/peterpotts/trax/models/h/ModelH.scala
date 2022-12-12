package com.peterpotts.trax.models.h

import com.peterpotts.trax.charting.Labels
import com.peterpotts.trax.charting.Suffices._
import com.peterpotts.trax.kalman._
import com.peterpotts.trax.math._
import com.peterpotts.trax.models._

/**
  * @author Peter Potts
  */
object ModelH extends Model3D {
  val title = "Model H"

  val stateLabels: Seq[String] = Labels(
    "d" -> V3,
    "v" -> V3,
    "a" -> V3,
    "q" -> UQ,
    "omegaB" -> V3,
    "alphaB" -> V3
  )

  val controlLabels: Seq[String] = Labels(
    "FB" -> V3,
    "tauB" -> V3
  )

  val measureLabels: Seq[String] = Labels(
    "dis" -> V3,
    "vel" -> V3,
    "accB" -> V3,
    "magB" -> V3,
    "optB" -> V3,
    "gyrB" -> V3
  )

  val steps: Seq[Step] = {
    val dt = 1.0
    val mass = 10.0
    val inertiaB = DiagonalMatrix33(Vector3.constant(0.1))
    val north = Vector3.X

    val controller = ControllerH(mass, inertiaB, north, dt)
    val predictor = PredictorH(mass, inertiaB, dt)
    val updater = UpdaterH(north)

    val start = {
      val target = controller.initial
      val actual = target.x
      val z = controller.measurements(actual, updater.v)
      val P = MatrixD.zero(XH.n, XH.n)
      val estimate = Estimate(actual, P)
      Step(0, actual, target.x, estimate, z, target.u)
    }

    val filter = ExtendedKalmanFilter(XH.n, UH.n, ZH.n, predictor, updater)

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

  def toEulerAngles(step: Step): EulerAngles = step.estimate.x.q.eulerAngles
}
