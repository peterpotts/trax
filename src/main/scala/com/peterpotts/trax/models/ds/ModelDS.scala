package com.peterpotts.trax.models.ds

import com.peterpotts.trax.kalman._
import com.peterpotts.trax.math.VectorDDecorator._
import com.peterpotts.trax.math._
import com.peterpotts.trax.models._

import scala.math._

/**
  * @author Peter Potts
  */
object ModelDS extends Model {
  val title = "Model DS"
  val stateLabels: Seq[String] = Seq("L", "theta", "a", "b", "c", "d")
  val controlLabels: Seq[String] = Seq("dummy")
  val measureLabels: Seq[String] = Seq("lif", "rot")

  val steps: Seq[Step] = {
    val dt = 1.0
    val weight = 0.0

    val controller: Controller = new Controller {
      val initial: Target = {
        //val x: X = XDS(L = 0, theta = 0, a = 0.002, b = -31, c = 0.002, d = -31)
        val x: X = XDS(L = 0, theta = 0, a = 0.001, b = -50, c = 0.001, d = -50)
        val u: U = UDS(dummy = 0)
        Target(x, u)
      }

      def target(time: Double, x: X, dt: Double): Target = initial

      private val iterator = DataDS.values.iterator

      def measurements(x: X, v: Z): Z = {
        val value = iterator.next()
        ZDS(lif = value, rot = value) fuzz v
      }

      def regulate(x: X, target: Target): U = UDS(dummy = 0)
    }

    val predictor: Predictor = new Predictor {
      val w: U = UDS(dummy = 0)

      def predict(x: X, u: U): Prediction =
        Prediction(
          f = x,
          F = MatrixD.identity(XDS.n),
          G = MatrixD.zero(XDS.n, UDS.n)
        )
    }

    val updater: Updater = new Updater {
      val v: Z = ZDS(
        lif = 3051,
        rot = 3051
      )

      def update(x: X): Update =
        Update(
          h = ZDS(
            lif = x.a * (x.L - weight * cos(x.theta)) + x.b,
            rot = x.c * x.L * sin(x.theta) + x.d
          ),
          H = ZDS.columns(
            lif = XDS.row(
              L = Vector1(x.a),
              theta = Vector1(x.a * weight * sin(x.theta)),
              a = Vector1(x.L - weight * cos(x.theta)),
              b = Vector1.identity
            ),
            rot = XDS.row(
              L = Vector1(x.c * sin(x.theta)),
              theta = Vector1(x.c * x.L * cos(x.theta)),
              c = Vector1(x.L * sin(x.theta)),
              d = Vector1.identity
            )
          )
        )
    }

    val start = {
      val target = controller.initial
      val actual = target.x
      val z = controller.measurements(actual, updater.v)

      val P = MatrixD.diagonal(XDS(
        L = 10000,
        theta = 0.1,
        a = 0.001,
        b = 10,
        c = 0.001,
        d = 10
      ).vectorD.squared)

      val estimate = Estimate(actual, P)
      Step(0, actual, target.x, estimate, z, target.u)
    }

    val filter = ExtendedKalmanFilter(XDS.n, UDS.n, ZDS.n, predictor, updater)

    def loop(step: Step): Step = {
      val actual = predictor.fuzzPredict(step.actual.x, step.u).f
      val target = controller.target(step.time, step.estimate.x, dt)
      val predict = filter.predict(step.u)(step.estimate)
      val z = controller.measurements(actual, updater.v)
      val update = filter.update(z)(predict)
      val u = controller.regulate(update.x, target)
      Step(step.time + dt, actual, target.x, update, z, u)
    }

    Iterator.iterate(start)(loop).take(DataDS.values.size).toSeq
  }
}
