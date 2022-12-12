package com.peterpotts.trax.fx

import scalafx.scene.input.{KeyCode, KeyEvent, MouseEvent}
import scalafx.scene.transform.Rotate
import scalafx.scene.{Group, PerspectiveCamera, Scene}

/**
  * @author Peter Potts
  */
class Viewpoint extends Group {
  private val camera = new PerspectiveCamera(true) {
    nearClip = 0.1
    farClip = 10000
    translateZ = -600
  }

  private val rotateZ: Rotate = new Rotate(-135, Rotate.ZAxis)
  private val rotateY: Rotate = new Rotate(0, Rotate.YAxis)
  private val rotateX: Rotate = new Rotate(65, Rotate.XAxis)

  children = camera
  transforms = List(rotateZ, rotateY, rotateX)

  def scene_=(scene: Scene): Unit = scene.camera = camera

  def onKeyPressed(keyEvent: KeyEvent): Unit = if (keyEvent.code == KeyCode.Z) {
    camera.translateZ = -600
    rotateZ.angle = -135
    rotateY.angle = 0
    rotateX.angle = 65
  }

  private var maybeLastMouseEvent: Option[MouseEvent] = None

  def onMousePressed(mouseEvent: MouseEvent): Unit = maybeLastMouseEvent = Some(mouseEvent)

  def onMouseReleased(mouseEvent: MouseEvent): Unit = maybeLastMouseEvent = None

  def onMouseDragged(mouseEvent: MouseEvent): Unit = {
    maybeLastMouseEvent.foreach { lastMouseEvent =>
      if (mouseEvent.isPrimaryButtonDown) {
        val mouseDeltaX = mouseEvent.sceneX - lastMouseEvent.sceneX
        val mouseDeltaY = mouseEvent.sceneY - lastMouseEvent.sceneY
        rotateX.angle = rotateX.angle.value - mouseDeltaY * 0.2
        rotateY.angle = rotateY.angle.value + mouseDeltaX * 0.2
      }

      maybeLastMouseEvent = Some(mouseEvent)
    }
  }

}
