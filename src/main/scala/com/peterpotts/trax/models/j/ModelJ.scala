package com.peterpotts.trax.models.j

import com.peterpotts.trax.charting.Labels
import com.peterpotts.trax.charting.Suffices._
import com.peterpotts.trax.kalman._
import com.peterpotts.trax.math._
import com.peterpotts.trax.models._

/**
  * @author Peter Potts
  */
object ModelJ extends Model3D {
  val title = "Model J"

  val stateLabels: Seq[String] = Labels(
    "d" -> V3,
    "v" -> V3,
    "a" -> V3,
    "r" -> V3,
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

    val controller = ControllerJ(mass, inertiaB, dt)
    val predictor = PredictorJ(mass, inertiaB, dt)
    val updater = UpdaterJ

    val start = {
      val target = controller.initial
      val actual = target.x
      val z = controller.measurements(actual, updater.v)
      val P = MatrixD.zero(XJ.n, XJ.n)
      val estimate = Estimate(actual, P)
      Step(0, actual, target.x, estimate, z, target.u)
    }

    val filter = ExtendedKalmanFilter(XJ.n, UJ.n, ZJ.n, predictor, updater)

    def loop(step: Step): Step = {
      val actual = predictor.fuzzPredict(step.actual.x, step.u).f
      val target = controller.target(step.time, step.estimate.x, dt)
      val predict = filter.predict(step.u)(step.estimate)
      val z = controller.measurements(actual, updater.v)
      val update = filter.update(z)(predict)
      val u = controller.regulate(update.x, target)
      Step(step.time + dt, actual, target.x, update, z, u)
    }

    Iterator.iterate(start)(loop).slice(0, 601).toSeq
  }

  def toEulerAngles(step: Step): EulerAngles = step.estimate.x.r.unitQuaternion.eulerAngles
}
