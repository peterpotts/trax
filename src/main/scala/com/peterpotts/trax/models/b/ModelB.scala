package com.peterpotts.trax.models.b

import com.peterpotts.trax.kalman._
import com.peterpotts.trax.math.MathDecorator._
import com.peterpotts.trax.math.VectorDDecorator._
import com.peterpotts.trax.math._
import com.peterpotts.trax.models.{Model, Step}

import scala.util.Random

/**
  * @author Peter Potts
  */
object ModelB extends Model {
  val title = "Model B"
  val stateLabels: Seq[String] = Seq("displacement", "velocity", "acceleration")
  val controlLabels: Seq[String] = Seq("force")
  val measureLabels: Seq[String] = Seq("accelerometer")

  val steps: Seq[Step] = {
    implicit val random: Random = new Random(0L)
    val dt = 1.0
    val mass = 10.0
    val displacement = 100.0
    val acceleration = -1.0
    val force = mass * acceleration
    val forceNoise = force * 0.1
    val accelerometerNoise = acceleration * 0.1

    val F = MatrixD(
      VectorD(1, dt, dt.squared / 2),
      VectorD(0, 1, dt),
      VectorD(0, 0, 0)
    )

    val G = MatrixD(
      VectorD(0),
      VectorD(0),
      VectorD(1 / mass)
    )

    val W = MatrixD(
      VectorD(0),
      VectorD(0),
      VectorD(1)
    )

    val H = MatrixD(VectorD(0, 0, 1))
    val w: U = UB(force = forceNoise)
    val v: Z = ZB(accelerometer = accelerometerNoise)
    val Q = MatrixD.diagonal(w.vectorD.squared)
    val R = MatrixD.diagonal(v.vectorD.squared)

    val filter = KalmanFilter(XB.n, UB.n, ZB.n, F, G, W, H, Q, R)

    def controller(time: Double): X = XB(
      displacement = displacement + acceleration * time.squared / 2,
      velocity = acceleration * time,
      acceleration = acceleration
    )

    //noinspection ScalaUnusedSymbol
    def uf(t: X): U = UB(force = force)

    def zf(t: X): Z =
      ZB(accelerometer = t.acceleration)

    val start = {
      val t = controller(0)

      val estimate = Estimate(
        x = t,
        P = MatrixD.constant(XB.n, XB.n, 1)
      )

      Step(0, t, t, estimate, zf(t) fuzz v, uf(t) fuzz w)
    }

    def loop(step: Step): Step = {
      val time = step.time + dt
      val t = controller(time)
      val u = uf(t) fuzz w
      val z = zf(t) fuzz v
      val predict = filter.predict(u)(step.estimate)
      val update = filter.update(z)(predict)
      Step(time, t, t, update, z, u)
    }

    Iterator.iterate(start)(loop).take(101).toSeq
  }
}
