package com.peterpotts.trax.math

import com.peterpotts.example.RandomExample._

import scala.math._

/**
  * @author Peter Potts
  */
object GeometryExamples {
  val examplePlusMinusPi: Example[Double] = exampleDouble.map(_ * Pi)
  val examplePlusTwoPi: Example[Double] = exampleDouble.map(_ * Pi + Pi)
  val examplePlusMinusHalfPi: Example[Double] = exampleDouble.map(_ * HalfPi)
  val examplePlusPi: Example[Double] = exampleDouble.map(_ * HalfPi + HalfPi)
  val examplePlusMinusTwoPi: Example[Double] = exampleDouble.map(_ * TwoPi)
  val examplePlusMinusThreePi: Example[Double] = exampleDouble.map(_ * ThreePi)
  val examplePlusMinusFourPi: Example[Double] = exampleDouble.map(_ * FourPi)
  val exampleNumber: Example[Double] = examplePlusMinusFourPi

  val exampleAngle: Example[Double] = exampleNumber.map(Angle.normalizePlusMinus)

  val exampleEulerAngles: Example[EulerAngles] =
    for {
      roll <- exampleNumber
      pitch <- exampleNumber
      yaw <- exampleNumber
    } yield EulerAngles.normalize(ϕ = roll, θ = pitch, ψ = yaw)

  val exampleQuaternion: Example[Quaternion] =
    for {
      w <- exampleNumber
      x <- exampleNumber
      y <- exampleNumber
      z <- exampleNumber
    } yield Quaternion(w = w, x = x, y = y, z = z)

  val exampleUnitQuaternion: Example[UnitQuaternion] = for (quaternion <- exampleQuaternion) yield quaternion.versor

  val exampleVector1: Example[Vector1] =
    for (x <- exampleNumber) yield Vector1(x)

  val exampleVector2: Example[Vector2] =
    for {
      x <- exampleNumber
      y <- exampleNumber
    } yield Vector2(x, y)

  val exampleVector3: Example[Vector3] =
    for {
      x <- exampleNumber
      y <- exampleNumber
      z <- exampleNumber
    } yield Vector3(x, y, z)

  val exampleVector4: Example[Vector4] =
    for {
      w <- exampleNumber
      x <- exampleNumber
      y <- exampleNumber
      z <- exampleNumber
    } yield Vector4(w, x, y, z)

  val exampleUnitVector: Example[Vector3] = for (vector3 <- exampleVector3) yield vector3 / vector3.norm

  val exampleAxisAngle: Example[AxisAngle] =
    for {
      n <- exampleUnitVector
      α <- exampleNumber
    } yield AxisAngle.normalize(n = n, α = α)

  val exampleRotationVector: Example[RotationVector] =
    for (vector3 <- exampleVector3) yield RotationVector.normalize(vector3)

  val exampleMatrix12: Example[Matrix12] =
    for (x <- exampleVector2) yield Matrix12(x)

  val exampleMatrix22: Example[Matrix22] =
    for {
      x <- exampleVector2
      y <- exampleVector2
    } yield Matrix22(x, y)

  val exampleDiagonalMatrix33: Example[DiagonalMatrix33] =
    for (vector3 <- exampleVector3) yield DiagonalMatrix33(vector3)

  val exampleMatrix33: Example[Matrix33] =
    for {
      x <- exampleVector3
      y <- exampleVector3
      z <- exampleVector3
    } yield Matrix33(x, y, z)

  val exampleMatrix34: Example[Matrix34] =
    for {
      x <- exampleVector4
      y <- exampleVector4
      z <- exampleVector4
    } yield Matrix34(x, y, z)

  val exampleMatrix43: Example[Matrix43] =
    for {
      w <- exampleVector3
      x <- exampleVector3
      y <- exampleVector3
      z <- exampleVector3
    } yield Matrix43(w, x, y, z)

  val exampleMatrix44: Example[Matrix44] =
    for {
      w <- exampleVector4
      x <- exampleVector4
      y <- exampleVector4
      z <- exampleVector4
    } yield Matrix44(w, x, y, z)

  val exampleTensor333: Example[Tensor333] =
    for {
      x <- exampleMatrix33
      y <- exampleMatrix33
      z <- exampleMatrix33
    } yield Tensor333(x, y, z)

  val exampleTensor334: Example[Tensor334] =
    for {
      x <- exampleMatrix34
      y <- exampleMatrix34
      z <- exampleMatrix34
    } yield Tensor334(x, y, z)

  val exampleTensor343: Example[Tensor343] =
    for {
      x <- exampleMatrix43
      y <- exampleMatrix43
      z <- exampleMatrix43
    } yield Tensor343(x, y, z)

  val exampleTensor433: Example[Tensor433] =
    for {
      w <- exampleMatrix33
      x <- exampleMatrix33
      y <- exampleMatrix33
      z <- exampleMatrix33
    } yield Tensor433(w, x, y, z)

  val exampleRotationMatrix: Example[RotationMatrix33] =
    for (unitQuaternion <- exampleUnitQuaternion) yield unitQuaternion.rotationMatrix

  def exampleVectorD(n: Int): Example[VectorD] = exampleVector(exampleDouble, n)

  def exampleMatrixD(n: Int, m: Int): Example[MatrixD] = exampleVector(exampleVectorD(m), n)
}
