package com.peterpotts.trax.mavlink.model

/**
  * @author Peter Potts
  */
object MAVDecoderApplication {
  def main(args: Array[String]): Unit = generate()

  private def generate(): Unit = {
    println("package com.peterpotts.trax.mavlink.model")
    println("")

    println("import com.peterpotts.trax.mavlink.model.MAVEnums._")
    println("import me.drton.jmavlib.mavlink.MAVLinkMessage")
    println("")
    println("/**")
    println("  * @author Peter Potts")
    println("  */")
    println("object MAVDecoder {")
    println("def apply(message: MAVLinkMessage): MAVMessage = {")
    println("val messages = Component(message)")
    println("")
    println("message.msgID match {")

    for (message <- MAVCommonMessages.document) {
      println(s"case ${message.id} =>")
      val messageClassName = MAVFormatter.toClassName(message.name)
      println(s"messages.$messageClassName(")

      val parameters =
        for {
          field <- message.fields
          if MAVVariable.isVariable(field)
        } yield {
          val variableName = MAVVariable.name(field)

          val variableValue =
            field.maybeEnum.fold {
              MAVVariable.get(field)
            } { enum =>
              val enumClassName = MAVFormatter.toClassName(enum)
              val getInt = s"""message.getInt("${field.name}")"""

              if (field.maybeDisplay.contains("bitmask")) {
                s"$enumClassName.bitmask($getInt)"
              } else {
                s"$enumClassName($getInt)"
              }
            }


          s"$variableName = $variableValue"
        }

      println(parameters.mkString(",\n"))
      println(")")
    }

    println("case error => throw new MatchError(error)")
    println("}")
    println("}")
    println("}")
  }
}
