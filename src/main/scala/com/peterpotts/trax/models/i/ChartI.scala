package com.peterpotts.trax.models.i

import com.peterpotts.trax.charting.ChartStage
import scalafx.application.JFXApp

/**
  * @author Peter Potts
  */
object ChartI extends JFXApp {
  stage = ChartStage(ModelI)
}
