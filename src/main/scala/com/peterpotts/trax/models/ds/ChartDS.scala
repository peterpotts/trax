package com.peterpotts.trax.models.ds

import com.peterpotts.trax.charting.ChartStage
import scalafx.application.JFXApp

/**
  * @author Peter Potts
  */
object ChartDS extends JFXApp {
  stage = ChartStage(ModelDS)
}
