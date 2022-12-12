package com.peterpotts.trax.math

import com.peterpotts.trax.math.MathDecorator._

import scala.math._

/**
  * @author Peter Potts
  */
case class Angle(θ: Double, validate: Boolean = true) {
  if (validate) {
    require(θ >= MinusPi, s"θ $θ must be greater than or equal to -π")
    require(θ < Pi, s"θ $θ must be less than π")
  }

  lazy val sθ: Double = sin(θ)
  lazy val cθ: Double = cos(θ)
  lazy val x: EulerAngles = EulerAngles.normalize(ϕ = θ, θ = 0, ψ = 0)
  lazy val y: EulerAngles = EulerAngles.normalize(ϕ = 0, θ = θ, ψ = 0)
  lazy val z: EulerAngles = EulerAngles.normalize(ϕ = 0, θ = 0, ψ = θ)

  lazy val vector1: Vector1 = Vector1(θ)

  def ===(that: Angle): Boolean = θ === that.θ

  def +(that: Angle): Angle = Angle.normalize(θ + that.θ)

  def -(that: Angle): Angle = Angle.normalize(θ - that.θ)

  /////////////////////
  // ROTATION MATRIX //
  /////////////////////

  lazy val rotationMatrix: RotationMatrix22 = RotationMatrix22(Matrix22(
    Vector2(cθ, sθ),
    Vector2(-sθ, cθ)
  ))

  lazy val jacobianRotationMatrix: Matrix22 = Matrix22(
    Vector2(-sθ, cθ),
    Vector2(-cθ, -sθ)
  )

  def integrateRotationMatrix(that: Angle): Matrix22 = jacobianRotationMatrix - that.jacobianRotationMatrix
}

object Angle {
  val zero: Angle = Angle(θ = 0)
  val right: Angle = Angle(θ = HalfPi)

  def apply(vector1: Vector1): Angle = Angle(vector1.value)

  def normalize(vector1: Vector1): Angle = normalize(vector1.value)

  def normalize(θ: Double): Angle = Angle(normalizePlusMinus(θ))

  /**
    * θ ∈ [-π, π)
    */
  def normalizePlusMinus(θ: Double): Double = normalizePlus(θ + Pi) - Pi

  /**
    * θ ∈ [0, 2π)
    */
  def normalizePlus(θ: Double): Double = {
    val value = θ % TwoPi
    if (value < 0.0) TwoPi + value else value
  }
}
