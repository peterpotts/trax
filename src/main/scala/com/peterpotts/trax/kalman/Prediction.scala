package com.peterpotts.trax.kalman

import com.peterpotts.trax.math.MatrixD

/**
  * @author Peter Potts
  */
case class Prediction(f: X, F: MatrixD, G: MatrixD)
