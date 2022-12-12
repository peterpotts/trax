package com.peterpotts.trax.math

import com.peterpotts.trax.math.MathDecorator._

/**
  * @author Peter Potts
  */
case class UnitQuaternion(quaternion: Quaternion, validate: Boolean) {

  import quaternion.{w, x, y, z}

  if (validate) {
    require(quaternion.norm === 1, s"norm ${quaternion.norm} must be one")
    require(!quaternion.isNegative, s"w ${quaternion.w} must be non-negative")
  }

  private lazy val ws: Double = w.squared
  private lazy val xs: Double = x.squared
  private lazy val ys: Double = y.squared
  private lazy val zs: Double = z.squared

  lazy val isLimit: Boolean = w > UnitQuaternion.upperBound
  lazy val inverse: UnitQuaternion = UnitQuaternion(quaternion.conjugate)

  def ===(that: UnitQuaternion): Boolean = quaternion === that.quaternion

  def ∘(that: UnitQuaternion): UnitQuaternion = UnitQuaternion(quaternion ∘ that.quaternion)

  ////////////////
  // AXIS ANGLE //
  ////////////////

  /**
    * Page 18 (198, 199)
    * Mathematica (nq, αq)
    *
    * | n_q(q)
    * | α_q(q)
    */
  lazy val axisAngle: AxisAngle =
    if (quaternion.length == 0)
      AxisAngle.zero
    else
      AxisAngle(
        n = quaternion.vector3.versor,
        α = 2 * arcCos(w)
      )

  ///////////////////////
  // QUATERNION MATRIX //
  ///////////////////////

  def quaternionMatrix: QuaternionMatrix = quaternion.quaternionMatrix

  def conjugateQuaternionMatrix: QuaternionMatrix = quaternion.conjugateQuaternionMatrix

  /////////////////////
  // ROTATION MATRIX //
  /////////////////////

  /**
    * Page 15 (125)
    * Mathematica (Rq)
    *
    * | R_q(q)
    */
  lazy val rotationMatrix: RotationMatrix33 = RotationMatrix33(Matrix33(
    Vector3(ws + xs - ys - zs, 2 * x * y + 2 * w * z, 2 * x * z - 2 * w * y),
    Vector3(2 * x * y - 2 * w * z, ws - xs + ys - zs, 2 * y * z + 2 * w * x),
    Vector3(2 * x * z + 2 * w * y, 2 * y * z - 2 * w * x, ws - xs - ys + zs)
  ))

  /**
    * Jacobian of rotation matrix with respect to unit quaternion.
    * Page 15 (129, 130)
    * Mathematica (dRqdq)
    *
    * | dR_q{ij}
    * | --------
    * |  dq{k}
    */
  lazy val jacobianRotationMatrix: Tensor334.IJK = Tensor334.IJK(Tensor334(
    Matrix34(
      Vector4(w, x, -y, -z),
      Vector4(z, y, x, w),
      Vector4(-y, z, -w, x)
    ),
    Matrix34(
      Vector4(-z, y, x, -w),
      Vector4(w, -x, y, -z),
      Vector4(x, w, z, y)
    ),
    Matrix34(
      Vector4(y, z, w, x),
      Vector4(-x, -w, z, y),
      Vector4(w, -x, -y, z)
    )
  ) * 2)

  //////////////////
  // RATES MATRIX //
  //////////////////

  /**
    * Page 16 (150)
    *
    * | W(q)
    */
  lazy val ratesMatrix: Matrix34 = Matrix34(
    Vector4(-x, w, -z, y),
    Vector4(-y, z, w, -x),
    Vector4(-z, -y, x, w)
  )

  /**
    * Page 16 (151)
    *
    * | W'(q)
    */
  lazy val conjugateRatesMatrix: Matrix34 = Matrix34(
    Vector4(-x, w, z, -y),
    Vector4(-y, -z, w, x),
    Vector4(-z, y, -x, w)
  )

  //////////////////
  // EULER ANGLES //
  //////////////////

  /**
    * Page 24 (290)
    * Mathematica (uq)
    *
    * | u_123(R_q(q))
    */
  lazy val eulerAngles: EulerAngles = EulerAngles.normalize(
    ϕ = arcTan2(2 * w * x + 2 * y * z, ws - xs - ys + zs),
    θ = arcSin(2 * w * y - 2 * x * z),
    ψ = arcTan2(2 * x * y + 2 * w * z, ws + xs - ys - zs)
  )

  /**
    * Mathematica (duqdq)
    */
  lazy val jacobianEulerAngles: Matrix34 = {
    val a = -ws + xs + ys - zs
    val b = -ws - xs + ys + zs
    val c = w * x + y * z
    val d = x * y + w * z

    Matrix34(
      Vector4(
        2 * w * c + x * a,
        -2 * x * c + w * a,
        -2 * y * c + z * a,
        2 * z * c + y * a
      ) / (4 * c.squared + (ws - xs - ys + zs).squared),
      Vector4(y, -z, w, -x) / squareRoot(1 - 4 * (w * y - x * z).squared),
      Vector4(
        2 * w * d + z * b,
        2 * x * d + y * b,
        -2 * y * d + x * b,
        -2 * z * d + w * b
      ) / (4 * d.squared + (ws + xs - ys - zs).squared)
    ) * 2
  }

  /////////////////////
  // ROTATION VECTOR //
  /////////////////////

  /**
    * Page 19 (221, 224)
    * Mathematica (vq)
    *
    * | v_q(q)
    */
  lazy val rotationVector: RotationVector = if (isLimit) limitRotationVector else axisAngle.rotationVector

  /**
    * Page 19 (224)
    * Mathematica (Lvq)
    */
  lazy val limitRotationVector: RotationVector = RotationVector(quaternion.vector3 * 2)

  /**
    * Jacobian of rotation vector with respect to unit quaternion.
    * Page 19 (225, 226, 227, 228)
    * Mathematica (dvqdq)
    *
    * |        dv_q{i}
    * | H(q) = -------
    * |         dq{j}
    */
  lazy val jacobianRotationVector: Matrix34 = if (isLimit) limitJacobianRotationVector else unsafeJacobianRotationVector

  lazy val unsafeJacobianRotationVector: Matrix34 = {
    val c = 1 / quaternion.length
    val d = arcCos(w) / squareRoot(quaternion.length)
    val e = 2 * c * (d * w - 1)

    Matrix34(
      Vector4(e * x, 2 * d, 0, 0),
      Vector4(e * y, 0, 2 * d, 0),
      Vector4(e * z, 0, 0, 2 * d)
    )
  }

  /**
    * Page 19 (228)
    * Mathematica (Ldvqdq)
    */
  lazy val limitJacobianRotationVector: Matrix34 =
    Matrix34(
      Vector4(0, 2, 0, 0),
      Vector4(0, 0, 2, 0),
      Vector4(0, 0, 0, 2)
    )
}

object UnitQuaternion {
  val W: UnitQuaternion = UnitQuaternion(Quaternion(1, 0, 0, 0))
  val X: UnitQuaternion = UnitQuaternion(Quaternion(0, 1, 0, 0))
  val Y: UnitQuaternion = UnitQuaternion(Quaternion(0, 0, 1, 0))
  val Z: UnitQuaternion = UnitQuaternion(Quaternion(0, 0, 0, 1))

  val I: UnitQuaternion = UnitQuaternion.W
  val X90: UnitQuaternion = AxisAngle(Vector3.X, 90.toRadians).unitQuaternion
  val X180: UnitQuaternion = AxisAngle(Vector3.X, 180.toRadians).unitQuaternion
  val X270: UnitQuaternion = AxisAngle(Vector3.X, 270.toRadians).unitQuaternion
  val Y90: UnitQuaternion = AxisAngle(Vector3.Y, 90.toRadians).unitQuaternion
  val Y180: UnitQuaternion = AxisAngle(Vector3.Y, 180.toRadians).unitQuaternion
  val Y270: UnitQuaternion = AxisAngle(Vector3.Y, 270.toRadians).unitQuaternion
  val Z90: UnitQuaternion = AxisAngle(Vector3.Z, 90.toRadians).unitQuaternion
  val Z180: UnitQuaternion = AxisAngle(Vector3.Z, 180.toRadians).unitQuaternion
  val Z270: UnitQuaternion = AxisAngle(Vector3.Z, 270.toRadians).unitQuaternion

  val zero: UnitQuaternion = W
  val upperBound: Double = 1 - AxisAngle.limit.squared / 8

  def apply(quaternion: Quaternion): UnitQuaternion =
    if (quaternion.isNegative)
      UnitQuaternion(quaternion.negate, validate = true)
    else
      UnitQuaternion(quaternion, validate = true)
}
