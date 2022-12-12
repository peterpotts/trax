package com.peterpotts.trax.fx

import scalafx.scene.Group
import scalafx.scene.input.{KeyCode, KeyEvent}
import scalafx.scene.shape.Box

/**
  * @author Peter Potts
  */
class Axes extends Group(
  new Box(400, 1, 1) {
    material = Material.DarkRed
  },
  new Box(1, 400, 1) {
    material = Material.LightGreen
  },
  new Box(1, 1, 400) {
    material = Material.DarkBlue
  },
  new Label("X") {
    translateX = 190
  },
  new Label("Y") {
    translateY = 190
  },
  new Label("Z") {
    translateZ = 190
  }
) {
  visible = true

  def onKeyPressed(keyEvent: KeyEvent): Unit = if (keyEvent.code == KeyCode.X) visible = !visible.value
}
