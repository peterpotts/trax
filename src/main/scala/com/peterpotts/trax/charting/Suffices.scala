package com.peterpotts.trax.charting

/**
  * @author Peter Potts
  */
sealed abstract class Suffices(val values: String*)

object Suffices {

  case object V1 extends Suffices()

  case object VZ extends Suffices("z")

  case object V2 extends Suffices("x", "y")

  case object V3 extends Suffices("x", "y", "z")

  case object UQ extends Suffices("w", "x", "y", "z")

  case object EA extends Suffices("ϕ", "θ", "ψ")

}
