package com.peterpotts.trax.common

/**
  * @author Peter Potts
  */
object FlattenedList {
  def apply(elements: Any*): List[String] = elements.flatMap(map(0)).to(List)

  private def map(level: Int)(elements: Any): List[String] =
    elements match {
      case option: Option[_] => option.toList.flatMap(map(level))
      case iterable: Iterable[_] => iterable.flatMap(map(level + 1)).to(List)
      case array: Array[_] => array.flatMap(map(level + 1)).to(List)
      case element => List(" " * level + element.toString)
    }
}
