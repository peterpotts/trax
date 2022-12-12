package com.peterpotts.trax.math

import com.peterpotts.trax.math.MathDecorator._

import scala.math._

/**
  * Euler angles are three angles to describe the orientation of a rigid body with respect to a fixed coordinate system.
  *
  * Using the Tait–Bryan convention, an aircraft points in the X direction with Y to the right and Z straight down.
  *
  * @param ϕ Roll [X].
  * @param θ Pitch [Y].
  * @param ψ Yaw [Z].
  * @author Peter Potts
  */
case class EulerAngles(ϕ: Double, θ: Double, ψ: Double, validate: Boolean) {
  if (validate) {
    require(ϕ >= MinusPi, s"ϕ $ϕ must be greater than or equal to -π")
    require(ϕ < Pi, s"ϕ $ϕ must be less than π")
    require(θ >= MinusHalfPi, s"θ $θ must be greater than or equal to -π/2")
    require(θ <= HalfPi, s"θ $θ must be less than or equal to π/2")
    require(ψ >= MinusPi, s"ψ $ψ must be greater than or equal to -π")
    require(ψ < Pi, s"ψ $ψ must be less than π")
  }

  private lazy val sϕ = sin(ϕ)
  private lazy val cϕ = cos(ϕ)
  private lazy val sθ = sin(θ)
  private lazy val cθ = cos(θ)
  private lazy val tθ = sθ / cθ
  private lazy val sψ = sin(ψ)
  private lazy val cψ = cos(ψ)

  private lazy val shϕ = sin(ϕ / 2)
  private lazy val chϕ = cos(ϕ / 2)
  private lazy val shθ = sin(θ / 2)
  private lazy val chθ = cos(θ / 2)
  private lazy val shψ = sin(ψ / 2)
  private lazy val chψ = cos(ψ / 2)

  lazy val vector3: Vector3 = Vector3(ϕ, θ, ψ)
  lazy val gimbalMeasure: Double = cos(θ)
  lazy val isGimbalLocked: Boolean = gimbalMeasure < 0.01
  lazy val inverse: EulerAngles = unitQuaternion.inverse.eulerAngles

  def ===(that: EulerAngles): Boolean = vector3 === that.vector3

  def +(that: EulerAngles): EulerAngles = (unitQuaternion ∘ that.unitQuaternion).eulerAngles

  /////////////////////
  // ROTATION MATRIX //
  /////////////////////

  /**
    * Page 24 (287)
    * Mathematica (R)
    *
    * | R_123(ϕ, θ, ψ)
    */
  lazy val rotationMatrix: RotationMatrix33 = RotationMatrix33(Matrix33(
    Vector3(cθ * cψ, cθ * sψ, -sθ),
    Vector3(sϕ * sθ * cψ - cϕ * sψ, sϕ * sθ * sψ + cϕ * cψ, cθ * sϕ),
    Vector3(cϕ * sθ * cψ + sϕ * sψ, cϕ * sθ * sψ - sϕ * cψ, cθ * cϕ)
  ))

  /**
    * Page 11 (68)
    * Mathematica (LR)
    *
    * | L{R_123(ϕ, θ, ψ)}
    */
  lazy val limitRotationMatrix: RotationMatrix33 = RotationMatrix33(Matrix33(
    Vector3(1, ψ, -θ),
    Vector3(-ψ, 1, ϕ),
    Vector3(θ, -ϕ, 1)
  ))

  /**
    * Jacobian of rotation matrix with respect to Euler angles.
    * Page 12 (69, 70, 71)
    * Mathematica (dRdu)
    *
    * | dR_123{ij}
    * | --------
    * |  du{k}
    */
  lazy val jacobianRotationMatrix: Tensor333.IJK = Tensor333.IJK(Tensor333(
    Matrix33(
      Vector3(0, -cψ * sθ, -cθ * sψ),
      Vector3(0, -sθ * sψ, cθ * cψ),
      Vector3(0, -cθ, 0)
    ),
    Matrix33(
      Vector3(cϕ * cψ * sθ + sϕ * sψ, cθ * cψ * sϕ, -cϕ * cψ - sθ * sϕ * sψ),
      Vector3(-cψ * sϕ + cϕ * sθ * sψ, cθ * sϕ * sψ, cψ * sθ * sϕ - cϕ * sψ),
      Vector3(cθ * cϕ, -sθ * sϕ, 0)
    ),
    Matrix33(
      Vector3(-cψ * sθ * sϕ + cϕ * sψ, cθ * cϕ * cψ, cψ * sϕ - cϕ * sθ * sψ),
      Vector3(-cϕ * cψ - sθ * sϕ * sψ, cθ * cϕ * sψ, cϕ * cψ * sθ + sϕ * sψ),
      Vector3(-cθ * sϕ, -cϕ * sθ, 0)
    )
  ))

  //////////////////
  // RATES MATRIX //
  //////////////////

  /**
    * Page 24 (291)
    * Mathematica (Eu)
    *
    * | E_123(ϕ, θ, ψ)
    */
  lazy val ratesMatrix: Matrix33 = Matrix33(
    Vector3(cθ * cψ, -sψ, 0),
    Vector3(cθ * sψ, cψ, 0),
    Vector3(-sθ, 0, 1)
  )

  /**
    * Page 24 (293)
    * Mathematica (cEu)
    *
    * | E'_123(ϕ, θ, ψ)
    */
  lazy val conjugateRatesMatrix: Matrix33 = Matrix33(
    Vector3(1, 0, -sθ),
    Vector3(0, cϕ, cθ * sϕ),
    Vector3(0, -sϕ, cθ * cϕ)
  )

  /**
    * Page 24 (296)
    * Mathematica (icEu)
    *
    * | E'_123(ϕ, θ, ψ)-1
    */
  lazy val inverseConjugateRatesMatrix: Matrix33 = Matrix33(
    Vector3(1, sϕ * tθ, cϕ * tθ),
    Vector3(0, cϕ, -sϕ),
    Vector3(0, sϕ / cθ, cϕ / cθ)
  )

  /**
    * Jacobian of rotation matrix with respect to Euler angles.
    * Page 12 (69, 70, 71)
    * Mathematica (dicEudu)
    *
    * | dE'_123(ϕ, θ, ψ)-1{ij}
    * | ----------------------
    * |  du{k}
    */
  lazy val jacobianInverseConjugateRatesMatrix: Tensor333.IJK = Tensor333.IJK(Tensor333(
    Matrix33(
      Vector3(0, 0, 0),
      Vector3(cϕ * tθ, sϕ / cθ.squared, 0),
      Vector3(-sϕ * tθ, cϕ / cθ.squared, 0)
    ),
    Matrix33(
      Vector3(0, 0, 0),
      Vector3(-sϕ, 0, 0),
      Vector3(-cϕ, 0, 0)
    ),
    Matrix33(
      Vector3(0, 0, 0),
      Vector3(cϕ / cθ, sϕ * tθ / cθ, 0),
      Vector3(-sϕ / cθ, cϕ * tθ / cθ, 0)
    )
  ))

  /////////////////////
  // UNIT QUATERNION //
  /////////////////////

  /**
    * Page 24 (297)
    * Mathematica (qu)
    *
    * | q_123(ϕ, θ, ψ)
    */
  lazy val unitQuaternion: UnitQuaternion = UnitQuaternion(Quaternion(
    w = chθ * chϕ * chψ + shθ * shϕ * shψ,
    x = chθ * chψ * shϕ - chϕ * shθ * shψ,
    y = chϕ * chψ * shθ + chθ * shϕ * shψ,
    z = -chψ * shθ * shϕ + chθ * chϕ * shψ
  ))

  /**
    * Mathematica (Lqu)
    *
    * | L{q_123(ϕ, θ, ψ)}
    */
  lazy val limitUnitQuaternion: UnitQuaternion = Quaternion(w = 1, x = ϕ / 2, y = θ / 2, z = ψ / 2).versor

  /**
    * Mathematica (dqudu)
    *
    * | dq_123
    * | ------
    * |  du
    */
  lazy val jacobianUnitQuaternion: Matrix43 =
    Matrix43(
      Vector3(
        -chθ * chψ * shϕ + chϕ * shθ * shψ,
        -chϕ * chψ * shθ + chθ * shϕ * shψ,
        chψ * shθ * shϕ - chθ * chϕ * shψ
      ),
      Vector3(
        chθ * chϕ * chψ + shθ * shϕ * shψ,
        -chψ * shθ * shϕ - chθ * chϕ * shψ,
        -chϕ * chψ * shθ - chθ * shϕ * shψ
      ),
      Vector3(
        -chψ * shθ * shϕ + chθ * chϕ * shψ,
        chθ * chϕ * chψ - shθ * shϕ * shψ,
        chθ * chψ * shϕ - chϕ * shθ * shψ
      ),
      Vector3(
        -chϕ * chψ * shθ - chθ * shϕ * shψ,
        -chθ * chψ * shϕ - chϕ * shθ * shψ,
        chθ * chϕ * chψ + shθ * shϕ * shψ
      )
    ) / 2

  @transient lazy val display: String = List(ϕ, θ, ψ).map(_.toDegrees).mkString("EulerAngles(", ", ", ")")

  override def toString: String = List(ϕ, θ, ψ).mkString("EulerAngles(", ", ", ")")
}

object EulerAngles {
  val zero: EulerAngles = EulerAngles(ϕ = 0, θ = 0, ψ = 0)

  def apply(vector3: Vector3): EulerAngles = EulerAngles(vector3.x, vector3.y, vector3.z)

  def normalize(vector3: Vector3): EulerAngles = normalize(vector3.x, vector3.y, vector3.z)

  def normalize(ϕ: Double, θ: Double, ψ: Double): EulerAngles =
    EulerAngles(Angle.normalizePlusMinus(ϕ), Angle.normalizePlusMinus(θ), Angle.normalizePlusMinus(ψ))

  def apply(ϕ: Double, θ: Double, ψ: Double): EulerAngles =
    if (θ.abs <= HalfPi)
      EulerAngles(ϕ, θ, ψ, validate = true)
    else
      EulerAngles(
        if (ϕ >= 0) ϕ - Pi else ϕ + Pi,
        if (θ > 0) Pi - θ else MinusPi - θ,
        if (ψ >= 0) ψ - Pi else ψ + Pi,
        validate = true
      )
}
