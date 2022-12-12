package com.peterpotts.trax.fx

import com.peterpotts.trax.math._
import scalafx.scene.Group
import scalafx.scene.input.{KeyCode, KeyEvent}

/**
  * @author Peter Potts
  */
class ManualOmega(group: Group) extends Aerodynamics(group) {
  private val δt = 0.1

  private var unitQuaternion = UnitQuaternion.zero

  def onRotate(vector3: Vector3): Unit = {
    val ω = vector3
    val δθ = ω * δt
    unitQuaternion = δθ.eulerAngles.limitUnitQuaternion ∘ unitQuaternion
    rotate = unitQuaternion.axisAngle.α.toDegrees
    rotationAxis = unitQuaternion.axisAngle.n.point
  }

  def onKeyPressed(keyEvent: KeyEvent): Unit =
    keyEvent.code match {
      case KeyCode.P =>
        val eulerAngles = unitQuaternion.eulerAngles
        println(s"Yaw: ${eulerAngles.ψ.toDegrees}, Pitch: ${eulerAngles.θ.toDegrees}, Roll: ${eulerAngles.ϕ.toDegrees}")
      case KeyCode.R =>
        unitQuaternion = UnitQuaternion.zero
      case _ =>
    }
}
