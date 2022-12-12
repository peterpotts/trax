package com.peterpotts.trax.fx

import com.peterpotts.trax.math._
import scalafx.scene.Group
import scalafx.scene.input.KeyEvent

/**
  * @author Peter Potts
  */
abstract class Aerodynamics(group: Group) extends Group(group) {
  /**
    * Rotational input values from -1 to +1 for the X, Y and X axes.
    */
  def onRotate(vector3: Vector3): Unit

  def onKeyPressed(keyEvent: KeyEvent): Unit
}
