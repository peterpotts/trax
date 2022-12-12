package com.peterpotts.trax.mavlink.model

import java.io.File

import com.peterpotts.trax.mavlink.model.DOMDecorator._
import javax.xml.parsers.DocumentBuilderFactory
import org.w3c.dom.Element

/**
  * @author Peter Potts
  */
object MAVCommonEnums {
  val document: IndexedSeq[MAVEnumElement] = {
    val xmlFile = new File(MAVSchema.xmlFileName)
    val xmlBuilder = DocumentBuilderFactory.newInstance.newDocumentBuilder
    val doc = xmlBuilder.parse(xmlFile)
    doc.getDocumentElement.normalize()
    val root: Element = doc.getDocumentElement
    require(root.getNodeName == "mavlink")
    val enumsElements = root.get("enums")
    val enumElements = enumsElements.flatMap(_.get("enum"))
    for (enumElement <- enumElements) yield MAVEnumElement(enumElement)
  }

  private val nameToEntries =
    document.map(enum => enum.name -> enum.entries.map(entry => entry.name -> entry.value).toMap).toMap

  def apply(name: String): Map[String, Int] = nameToEntries(name)
}
