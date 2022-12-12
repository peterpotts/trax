package com.peterpotts.trax.mavlink.model

/**
  * @author Peter Potts
  */
object MAVMessagesApplication {
  def main(args: Array[String]): Unit = generate()

  private def generate(): Unit = {
    println("package com.peterpotts.trax.mavlink.model")
    println("")
    println("import com.peterpotts.trax.mavlink.model.MAVEnums._")
    println("")
    println("/**")
    println("  * @author Peter Potts")
    println("  */")
    println("class MAVMessages(systemId: Int, componentId: MAVComponent, protocolVersion: Int) {")
    println("")

    for (message <- MAVCommonMessages.document) {
      println("/**")
      println(s"  * ${message.id} ${message.name}")

      for (description <- message.descriptions) {
        println(s"  *")
        println(s"  * $description")
      }

      println(s"  *")

      for {
        field <- message.fields
        if MAVVariable.isVariable(field)
      } {
        val variableName = MAVVariable.name(field)
        val variableDescription = MAVVariable.description(field)
        println(s"  * @param $variableName $variableDescription")
      }

      val messageClassName = MAVFormatter.toClassName(message.name)

      println("  */")
      println(s"case class $messageClassName(")

      val indexedSeq =
        for {
          field <- message.fields
          if MAVVariable.isVariable(field)
        } yield {
          val variableName = MAVVariable.name(field)
          val variableType = MAVVariable.`type`(field)
          s"""$variableName: $variableType"""
        }

      println(indexedSeq.mkString(",\n"))

      println(s""") extends MAVMessage("${message.name}", systemId, componentId, protocolVersion) {""")

      for {
        field <- message.fields
        if MAVVariable.isVariable(field)
      } {
        val variableValue = MAVVariable.value(field)
        println(s"""set("${field.name}", $variableValue)""")
      }

      println("")
      println("override def toString: String =")
      println("List(")

      val indexedSeq2 =
        for {
          field <- message.fields
          if MAVVariable.isVariable(field)
        } yield {
          if (field.name == "type") {
            """s"type = ${`type`}""""
          } else {
            val variableName = MAVVariable.name(field)
            s"""s"$variableName = $$$variableName""""
          }
        }

      println(indexedSeq2.mkString(",\n"))
      println(s""").mkString("$messageClassName(", ", ", ")")""")
      println("}")
    }

    println("}")
  }


}
