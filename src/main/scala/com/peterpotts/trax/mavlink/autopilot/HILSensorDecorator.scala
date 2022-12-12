package com.peterpotts.trax.mavlink.autopilot

import com.peterpotts.trax.mavlink.model.Vehicle

/**
  * @author Peter Potts
  */
object HILSensorDecorator {

  implicit class DecoratedHILSensor(val hILSensor: Vehicle.HILSensor) extends AnyVal {
    private def updated(index: Int): Boolean = ((hILSensor.fieldsUpdated >> index) & 1) == 1

    def maybeXAcc: Option[Float] = if (updated(0)) Some(hILSensor.xAcc) else None

    def maybeYAcc: Option[Float] = if (updated(1)) Some(hILSensor.yAcc) else None

    def maybeZAcc: Option[Float] = if (updated(2)) Some(hILSensor.zAcc) else None

    def maybeXGyro: Option[Float] = if (updated(3)) Some(hILSensor.xGyro) else None

    def maybeYGyro: Option[Float] = if (updated(4)) Some(hILSensor.yGyro) else None

    def maybeZGyro: Option[Float] = if (updated(5)) Some(hILSensor.zGyro) else None

    def maybeXMag: Option[Float] = if (updated(6)) Some(hILSensor.xMag) else None

    def maybeYMag: Option[Float] = if (updated(7)) Some(hILSensor.yMag) else None

    def maybeZMag: Option[Float] = if (updated(8)) Some(hILSensor.zMag) else None

    def maybeAbsPressure: Option[Float] = if (updated(9)) Some(hILSensor.absPressure) else None

    def maybeDiffPressure: Option[Float] = if (updated(10)) Some(hILSensor.diffPressure) else None

    def maybePressureAlt: Option[Float] = if (updated(11)) Some(hILSensor.pressureAlt) else None

    def maybeTemperature: Option[Float] = if (updated(12)) Some(hILSensor.temperature) else None
  }

}
