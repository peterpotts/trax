package com.peterpotts.trax.fx

import scalafx.scene.layout.HBox
import scalafx.scene.text.Text
import scalafx.scene.transform.Rotate

/**
  * @author Peter Potts
  */
class Label(value: String) extends HBox(
  new Text {
    text = value
    style = "-fx-font-size: 12pt"
    transforms = List(new Rotate(180, Rotate.ZAxis), new Rotate(90, Rotate.XAxis))
  }
)
