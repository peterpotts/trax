package com.peterpotts.trax.mavlink.model

import java.io.File

import com.peterpotts.trax.mavlink.model.DOMDecorator._
import javax.xml.parsers.DocumentBuilderFactory
import org.w3c.dom.Element

/**
  * @author Peter Potts
  */
object MAVCommonMessages {
  val document: IndexedSeq[MAVMessageElement] = {
    val xmlFile = new File(MAVSchema.xmlFileName)
    val xmlBuilder = DocumentBuilderFactory.newInstance.newDocumentBuilder
    val doc = xmlBuilder.parse(xmlFile)
    doc.getDocumentElement.normalize()
    val root: Element = doc.getDocumentElement
    require(root.getNodeName == "mavlink")
    val messagesElements = root.get("messages")
    val messageElements = messagesElements.flatMap(_.get("message"))
    for (messageElement <- messageElements) yield MAVMessageElement(messageElement)
  }

  private val nameToMessage: Map[String, MAVMessageElement] = document.map(message => message.name -> message).toMap

  def apply(name: String): MAVMessageElement = nameToMessage(name)
}
