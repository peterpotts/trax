package com.peterpotts.trax.mavlink.model

import com.peterpotts.trax.mavlink.model.DOMDecorator._
import org.w3c.dom.Element

/**
  * @author Peter Potts
  */
case class MAVEnumElement(name: String, descriptions: IndexedSeq[String], entries: IndexedSeq[MAVEntryElement])

object MAVEnumElement {
  private val nameToPrefix =
    Map(
      "MAV_COMPONENT" -> "MAV_COMP_",
      "MAV_SYS_STATUS_SENSOR" -> "MAV_SYS_STATUS_",
      "MAVLINK_DATA_STREAM_TYPE" -> "MAVLINK_DATA_STREAM_",
      "MAV_MISSION_RESULT" -> "MAV_MISSION_",
      "MAV_SENSOR_ORIENTATION" -> "MAV_SENSOR_",
      "MAV_BATTERY_FUNCTION" -> "MAV_BATTERY_",
      "ESTIMATOR_STATUS_FLAGS" -> "ESTIMATOR_",
      "MOTOR_TEST_THROTTLE_TYPE" -> "MOTOR_TEST_THROTTLE_",
      "GPS_INPUT_IGNORE_FLAGS" -> "GPS_INPUT_IGNORE_FLAG_"
    )

  def apply(element: Element): MAVEnumElement = {
    val descriptions = element.get("description").map(_.contents)
    val enumName = element.name
    val prefix = nameToPrefix.getOrElse(enumName, enumName + "_")
    var enumValue = 0

    val entries =
      element.get("entry").map { entry =>
        lazy val commonPrefix = entry.name.zip(enumName).takeWhile(Function.tupled(_ == _)).map(_._1).mkString
        require(entry.name.startsWith(prefix), s""""$enumName" -> "$commonPrefix"""")
        val suffix = entry.name.drop(prefix.length)
        enumValue = entry.valueOption.fold(enumValue + 1)(_.toInt)
        val descriptions = entry.get("description").map(_.contents)
        MAVEntryElement(enumValue, suffix, descriptions)
      }

    new MAVEnumElement(element.name, descriptions, entries)
  }
}
