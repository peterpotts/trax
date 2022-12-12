package com.peterpotts.trax.models.j

import com.peterpotts.trax.charting.ChartStage
import scalafx.application.JFXApp

/**
  * @author Peter Potts
  */
object ChartJ extends JFXApp {
  stage = ChartStage(ModelJ)
}
