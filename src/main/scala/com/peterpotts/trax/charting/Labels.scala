package com.peterpotts.trax.charting

/**
  * @author Peter Potts
  */
object Labels {
  def apply(tuples: (String, Suffices)*): Seq[String] = tuples.flatMap {
    case (prefix, suffices) =>
      if (suffices.values.isEmpty) Seq(prefix) else suffices.values.map(prefix + "." + _)
  }
}
