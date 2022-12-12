package com.peterpotts.trax.fx

import scalafx.scene.Group
import scalafx.scene.input.{KeyCode, KeyEvent}
import scalafx.scene.transform.Rotate

/**
  * @author Peter Potts
  */
class Drone extends Group {
  private val body = new Body

  private val frontLeft = new PropellerArm(Material.LightRed) {
    transforms = List(new Rotate(225, Rotate.ZAxis))
  }

  private val frontRight = new PropellerArm(Material.LightGreen) {
    transforms = List(new Rotate(315, Rotate.ZAxis))
  }

  private val backLeft = new PropellerArm(Material.LightBlue) {
    transforms = List(new Rotate(135, Rotate.ZAxis))
  }

  private val backRight = new PropellerArm(Material.LightBlue) {
    transforms = List(new Rotate(45, Rotate.ZAxis))
  }

  children = List(body, backRight, backLeft, frontLeft, frontRight)

  def onKeyPressed(keyEvent: KeyEvent): Unit = if (keyEvent.code == KeyCode.V) visible = !visible.value
}
