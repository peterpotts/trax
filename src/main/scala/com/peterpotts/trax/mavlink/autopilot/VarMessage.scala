package com.peterpotts.trax.mavlink.autopilot

import com.peterpotts.trax.mavlink.model.MAVMessage

/**
  * @author Peter Potts
  */
class VarMessage[T <: MAVMessage](observers: Int) extends Var[T](observers) {
  override def set(message: T): Unit = {
    val definition = message.definition
    val fields = definition.fields

    for ((field, index) <- fields.zipWithIndex) {
      val value = message.get(index)

      val formattedValue =
        value match {
          case array: Array[_] => array.mkString("[", ", ", "]")
          case _ => value.toString
        }

      val log = maybeLastMessage.fold(true)(_.get(index) != value)
      if (log) logger.info(s"${definition.name}.${field.name} = $formattedValue")
    }

    super.set(message)
  }
}
