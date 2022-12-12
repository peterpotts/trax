package com.peterpotts.trax.fx

import com.peterpotts.trax.usb.ManualController
import scalafx.Includes._
import scalafx.scene.Scene
import scalafx.scene.input.{KeyCode, KeyEvent, MouseEvent}
import scalafx.scene.paint.Color

/**
  * @author Peter Potts
  */
class Window(world: World, viewpoint: Viewpoint, remoteControl: ManualController) extends Scene(world, 1024, 768, true) {
  fill = Color.Gray

  onKeyPressed = { keyEvent: KeyEvent =>
    world.onKeyPressed(keyEvent)
    viewpoint.onKeyPressed(keyEvent)
    if (keyEvent.code == KeyCode.R) remoteControl.reset()
  }

  onMousePressed = { mouseEvent: MouseEvent => viewpoint.onMousePressed(mouseEvent) }
  onMouseReleased = { mouseEvent: MouseEvent => viewpoint.onMouseReleased(mouseEvent) }
  onMouseDragged = { mouseEvent: MouseEvent => viewpoint.onMouseDragged(mouseEvent) }
}
