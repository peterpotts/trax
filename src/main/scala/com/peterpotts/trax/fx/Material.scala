package com.peterpotts.trax.fx

import scalafx.scene.paint.{Color, PhongMaterial}

/**
  * @author Peter Potts
  */
object Material {
  val DarkRed: PhongMaterial = new PhongMaterial {
    diffuseColor = Color.DarkRed
    specularColor = Color.Red
  }

  val LightRed: PhongMaterial = new PhongMaterial {
    diffuseColor = Color.Red
    specularColor = Color.Pink
  }

  val DarkGreen: PhongMaterial = new PhongMaterial {
    diffuseColor = Color.DarkGreen
    specularColor = Color.Green
  }

  val LightGreen: PhongMaterial = new PhongMaterial {
    diffuseColor = Color.Green
    specularColor = Color.LightGreen
  }

  val DarkBlue: PhongMaterial = new PhongMaterial {
    diffuseColor = Color.DarkBlue
    specularColor = Color.Blue
  }

  val LightBlue: PhongMaterial = new PhongMaterial {
    diffuseColor = Color.Blue
    specularColor = Color.LightBlue
  }

  val White: PhongMaterial = new PhongMaterial {
    diffuseColor = Color.White
    specularColor = Color.LightBlue
  }

  val DarkGray: PhongMaterial = new PhongMaterial {
    diffuseColor = Color.DarkGray
    specularColor = Color.Gray
  }

  val LightGray: PhongMaterial = new PhongMaterial {
    diffuseColor = Color.Gray
    specularColor = Color.LightGray
  }
}
