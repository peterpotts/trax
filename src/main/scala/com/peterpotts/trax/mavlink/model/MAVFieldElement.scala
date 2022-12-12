package com.peterpotts.trax.mavlink.model

import com.peterpotts.trax.mavlink.model.DOMDecorator._
import org.w3c.dom.Element

/**
  * @author Peter Potts
  */
case class MAVFieldElement(
  `type`: String,
  name: String,
  maybeEnum: Option[String],
  maybeDisplay: Option[String],
  description: String
)

object MAVFieldElement {
  def apply(element: Element): MAVFieldElement =
    MAVFieldElement(
      element.`type`,
      element.name,
      element.enumOption,
      element.displayOption,
      element.contents
    )
}
