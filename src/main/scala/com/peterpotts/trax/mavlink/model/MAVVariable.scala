package com.peterpotts.trax.mavlink.model

/**
  * @author Peter Potts
  */
object MAVVariable {
  private val Single = "([a-z0-9_]*)".r
  private val Multiple = "([a-z0-9_]*)\\[[0-9]*]".r

  private val toScalaType =
    Map(
      "int8_t" -> "Int",
      "uint8_t" -> "Int",
      "int16_t" -> "Int",
      "uint16_t" -> "Int",
      "int32_t" -> "Int",
      "uint32_t" -> "Int",
      "int64_t" -> "Long",
      "uint64_t" -> "Long",
      "float" -> "Float",
      "double" -> "Double"
    )

  private val scala2Java =
    Map(
      "int8_t" -> "int2Integer",
      "uint8_t" -> "int2Integer",
      "int16_t" -> "int2Integer",
      "uint16_t" -> "int2Integer",
      "int32_t" -> "int2Integer",
      "uint32_t" -> "int2Integer",
      "int64_t" -> "long2Long",
      "uint64_t" -> "long2Long",
      "float" -> "float2Float",
      "double" -> "double2Double"
    )

  private val toJavaType =
    Map(
      "int8_t" -> "java.lang.Integer",
      "uint8_t" -> "java.lang.Integer",
      "int16_t" -> "java.lang.Integer",
      "uint16_t" -> "java.lang.Integer",
      "int32_t" -> "java.lang.Integer",
      "uint32_t" -> "java.lang.Integer",
      "int64_t" -> "java.lang.Long",
      "uint64_t" -> "java.lang.Long",
      "float" -> "java.lang.Float",
      "double" -> "java.lang.Double"
    )

  def isVariable(field: MAVFieldElement): Boolean = field.`type` != "uint8_t_mavlink_version"

  def name(field: MAVFieldElement): String = MAVFormatter.toVariableName(field.name)

  def description(field: MAVFieldElement): String = field.description.replace("^", "^^")

  def `type`(field: MAVFieldElement): String =
    field.maybeEnum.fold {
      field.`type` match {
        case "uint8_t_mavlink_version" => throw new MatchError("uint8_t_mavlink_version")
        case Single(fieldType) => toScalaType(fieldType)
        case Multiple("char") => "String"
        case Multiple(fieldType) => s"IndexedSeq[${toScalaType(fieldType)}]"
        case error => throw new MatchError(error)
      }
    } { enum =>
      val enumClassName = MAVFormatter.toClassName(enum)

      if (field.maybeDisplay.contains("bitmask"))
        s"Set[$enumClassName]"
      else
        enumClassName
    }

  def get(field: MAVFieldElement): String =
    field.`type` match {
      case "uint8_t_mavlink_version" => throw new MatchError("uint8_t_mavlink_version")
      case Single(fieldType) =>
        val scalaType = toScalaType(fieldType)
        s"""message.get$scalaType("${field.name}")"""
      case Multiple("char") => s"""message.getString("${field.name}")"""
      case Multiple(fieldType) =>
        val javaType = toJavaType(fieldType)
        val scalaType = toScalaType(fieldType)
        s"""message.get("${field.name}").asInstanceOf[Array[AnyRef]].map(_.asInstanceOf[$javaType].to$scalaType).toIndexedSeq"""
      case error => throw new MatchError(error)
    }

  def value(field: MAVFieldElement): String = {
    val variableName = name(field)

    field.maybeEnum.fold {
      field.`type` match {
        case "uint8_t_mavlink_version" => throw new MatchError("uint8_t_mavlink_version")
        case Single(_) => variableName
        case Multiple("char") => variableName
        case Multiple(fieldType) => s"$variableName.map(${scala2Java(fieldType)}).toArray"
        case error => throw new MatchError(error)
      }
    } { _ =>
      if (field.maybeDisplay.contains("bitmask"))
        s"$variableName.foldLeft(0)(_ | _.value)"
      else
        s"$variableName.value"
    }
  }
}
