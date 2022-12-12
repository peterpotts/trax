package com.peterpotts.trax.models

/**
  * @author Peter Potts
  */
trait Model {
  val title: String
  val stateLabels: Seq[String]
  val controlLabels: Seq[String]
  val measureLabels: Seq[String]
  val steps: Seq[Step]
}
