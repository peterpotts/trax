package com.peterpotts.trax.mavlink.model

import com.peterpotts.trax.mavlink.model.DOMDecorator._
import org.w3c.dom.Element

/**
  * @author Peter Potts
  */
case class MAVMessageElement(
  id: Int,
  name: String,
  descriptions: IndexedSeq[String],
  fields: IndexedSeq[MAVFieldElement]
)

object MAVMessageElement {
  def apply(element: Element): MAVMessageElement = {
    val descriptions = element.get("description").map(_.contents)
    val fields = element.get("field").map(MAVFieldElement(_))
    new MAVMessageElement(element.id, element.name, descriptions, fields)
  }
}
