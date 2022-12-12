package com.peterpotts.trax.mavlink.model

import org.w3c.dom.{Element, Node, NodeList}

/**
  * @author Peter Potts
  */
object DOMDecorator {

  implicit class DecoratedNodeList(val nodeList: NodeList) extends AnyVal {
    def nodes: IndexedSeq[Node] =
      for (index <- 0 until nodeList.getLength) yield nodeList.item(index)
  }

  implicit class DecoratedNodes(val nodes: IndexedSeq[Node]) extends AnyVal {
    def elements: IndexedSeq[Element] =
      nodes.collect {
        case element: Element => element
      }
  }

  implicit class DecoratedElement(val element: Element) extends AnyVal {
    def get(name: String): IndexedSeq[Element] = element.getChildNodes.nodes.elements.filter(_.getTagName == name)

    def attributeOption(name: String): Option[String] = {
      val attribute = element.getAttribute(name)
      if (attribute.isEmpty) None else Some(attribute)
    }

    def attribute(name: String): String = attributeOption(name).get

    def id: Int = attribute("id").toInt

    def `type`: String = attribute("type")

    def name: String = attribute("name")

    def valueOption: Option[String] = attributeOption("value")

    def enumOption: Option[String] = attributeOption("enum")

    def displayOption: Option[String] = attributeOption("display")

    def contents: String = element.getTextContent

    def elements: IndexedSeq[Element] = element.getChildNodes.nodes.elements
  }

}
