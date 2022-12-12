package com.peterpotts.trax.fx

import com.peterpotts.trax.math.MathDecorator._
import com.peterpotts.trax.math._
import scalafx.scene.Group
import scalafx.scene.input.{KeyCode, KeyEvent}

/**
  * Mass = 0.5 kg
  * Radius = 0.15 m
  * Input X = 1 => Torque X = 0.075 Nm
  * Input Y = 1 => Torque Y = 0.075 Nm
  * Input Z = 1 => Torque Z = 0.15 Nm
  *
  * @author Peter Potts
  */
class ManualTorque(group: Group) extends Aerodynamics(group) {
  private val δt = 0.1
  private val m = 0.5
  private val l = 0.15
  private val Ix = m * l.squared * 2 / 3
  private val Iy = Ix
  private val Iz = 2 * Ix
  private val I = Matrix33(Vector3(Ix, 0, 0), Vector3(0, Iy, 0), Vector3(0, 0, Iz))

  private var r = RotationVector.zero
  private var ω = Vector3(0, 0, 0)

  def onRotate(vector3: Vector3): Unit = {
    val τ = Vector3(vector3.x * 0.075, vector3.y * 0.075, vector3.z * 0.15)
    val α = I.inverse * (τ - ω × (I * ω))
    val δω = α * δt
    ω = ω + δω
    val δr = RotationVector.normalize(ω * δt)
    r = r * δr
    rotate = r.axisAngle.α.toDegrees
    rotationAxis = r.axisAngle.n.point
  }

  def onKeyPressed(keyEvent: KeyEvent): Unit =
    keyEvent.code match {
      case KeyCode.P =>
        val eulerAngles = r.unitQuaternion.eulerAngles
        println(s"Yaw: ${eulerAngles.ψ.toDegrees}, Pitch: ${eulerAngles.θ.toDegrees}, Roll: ${eulerAngles.ϕ.toDegrees}")
      case KeyCode.R =>
        r = RotationVector.zero
        ω = Vector3(0, 0, 0)
      case _ =>
    }
}
