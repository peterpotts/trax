package com.peterpotts.trax.mavlink.model

/**
  * @author Peter Potts
  */
abstract class MAVEnum(val name: String) {
  val nameToValue: Map[String, Int] = MAVCommonEnums(name)

  def apply(names: String*): Int = names.map(nameToValue).reduce(_ | _)
}
