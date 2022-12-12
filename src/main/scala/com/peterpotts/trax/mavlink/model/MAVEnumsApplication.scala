package com.peterpotts.trax.mavlink.model

/**
  * @author Peter Potts
  */
object MAVEnumsApplication {
  def main(args: Array[String]): Unit = generate()

  private def generate(): Unit = {
    println("package com.peterpotts.trax.mavlink.model")
    println("")
    println("/**")
    println("  * @author Peter Potts")
    println("  */")
    println("object MAVEnums {")

    for (enum <- MAVCommonEnums.document) {
      val enumClassName = MAVFormatter.toClassName(enum.name)
      println("/**")
      println(s"  * ${enum.name}")

      for (description <- enum.descriptions) {
        println(s"  *")
        println(s"  * $description")
      }

      println(s"  *")
      println(s"  * {{{")

      for (entry <- enum.entries) {
        val description = entry.descriptions.map(" " + _).mkString
        println(s"  * entry ${entry.value} ${entry.name}$description")
      }

      println("  }}}")
      println("  */")
      println(s"class $enumClassName(val value: Int)")
      println("")
      println(s"object $enumClassName {")

      for (entry <- enum.entries) {
        val entryClassName = MAVFormatter.toClassName(entry.name)
        println(s"case object $entryClassName extends $enumClassName(${entry.value})")
      }

      println("")
      println(s"val set: Set[$enumClassName] =")
      println("Set(")
      val indexedSeq = for (entry <- enum.entries) yield MAVFormatter.toClassName(entry.name)
      println(indexedSeq.mkString(",\n"))
      println(")")
      println("")
      println(s"val map: Map[Int, $enumClassName] = set.toSeq.map(instance => instance.value -> instance).toMap")
      println("")
      println(s"def apply(value: Int): $enumClassName = map.getOrElse(value, new $enumClassName(value))")
      println("")
      println(s"def bitmask(value: Int): Set[$enumClassName] = {")
      println("val known = set.filter(instance => (instance.value & value) == instance.value)")
      println("val sum = known.map(_.value).sum")
      println(s"if (value == sum) known else known + new $enumClassName(value - sum)")
      println(s"}")
      println("}")
    }

    println("}")
  }
}
