package com.peterpotts.trax.models.a

import com.peterpotts.trax.kalman._
import com.peterpotts.trax.math.MathDecorator._
import com.peterpotts.trax.math._
import com.peterpotts.trax.models._

/**
  * @author Peter Potts
  */
object ModelA extends Model {
  val title = "Model A"
  val stateLabels: Seq[String] = Seq("displacement", "velocity")
  val controlLabels: Seq[String] = Seq("force")
  val measureLabels: Seq[String] = Seq("displacement")

  val steps: Seq[Step] = {
    val dt = 1.0
    val mass = 1.0
    val displacement = 100.0
    val acceleration = -1.0
    val force = mass * acceleration

    val F = MatrixD(
      VectorD(1, dt),
      VectorD(0, 1)
    )

    val G = MatrixD(
      VectorD(dt.squared / (2 * mass)),
      VectorD(dt / mass)
    )

    val W = MatrixD(
      VectorD(0),
      VectorD(0)
    )

    val H = MatrixD(VectorD(1, 0))
    val Q = MatrixD.diagonal(VectorD(0))
    val R = MatrixD.diagonal(VectorD(1))

    val filter = KalmanFilter(XA.n, UA.n, ZA.n, F, G, W, H, Q, R)

    def controller(time: Double): X = XA(
      displacement = displacement + acceleration * time.squared / 2,
      velocity = acceleration * time
    )

    var t = controller(time = 0.0)

    var estimate = Estimate(
      x = XA(displacement = 95, velocity = 1),
      P = MatrixD.diagonal(VectorD(10, 1))
    )

    val u = UA(force = force)

    def z(step: Int) = ZA(displacement = Array(100.0, 100.0, 97.9, 94.4, 92.7, 87.3)(step))

    val head = Step(0, t, t, estimate, z(0), u)

    val tail = for (step <- 1 to 5) yield {
      val time = step * dt
      t = controller(step * dt)
      estimate = filter.predict(u)(estimate)
      estimate = filter.update(z(step))(estimate)
      Step(time, t, t, estimate, z(step), u)
    }

    head +: tail
  }
}
