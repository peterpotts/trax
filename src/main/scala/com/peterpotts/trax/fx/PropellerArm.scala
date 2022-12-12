package com.peterpotts.trax.fx

import scalafx.scene.Group
import scalafx.scene.paint.PhongMaterial
import scalafx.scene.transform.Rotate

/**
  * @author Peter Potts
  */
class PropellerArm(propellerMaterial: PhongMaterial) extends Group(
  new Propeller {
    material = propellerMaterial
    rotationAxis = Rotate.XAxis
    rotate = 90
    translateY = 100
  },
  new Arm {
    translateY = 50
  }
)
