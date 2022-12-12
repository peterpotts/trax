package com.peterpotts.trax.fx

import com.peterpotts.trax.math.{EulerAngles, Vector3}
import scalafx.scene._
import scalafx.scene.input._

/**
  * @author Peter Potts
  */
class World extends Group {
  private val axes = new Axes
  private val drone = new Drone
  private val aerodynamics: Aerodynamics = new ManualTorque(drone)
  //private val aerodynamics: Aerodynamics = new ManualOmega(drone)

  children = List(axes, aerodynamics)
  depthTest = DepthTest.Enable

  def setRotate(eulerAngles: EulerAngles): Unit = {
    aerodynamics.rotate = eulerAngles.unitQuaternion.axisAngle.α.toDegrees
    aerodynamics.rotationAxis = eulerAngles.unitQuaternion.axisAngle.n.point
  }

  def onRotate(vector3: Vector3): Unit = aerodynamics.onRotate(vector3)

  def onKeyPressed(keyEvent: KeyEvent): Unit = {
    axes.onKeyPressed(keyEvent)
    drone.onKeyPressed(keyEvent)
    aerodynamics.onKeyPressed(keyEvent)
  }
}
