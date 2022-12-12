package com.peterpotts.trax.mavlink.autopilot

import com.peterpotts.trax.mavlink.model._

/**
  * @author Peter Potts
  */
case class GroundControlParamIO(write: MAVMessage => Unit) extends AutopilotGroundControl {
  private val paramRequestList =
    GroundControl.ParamRequestList(
      targetSystem = systemId,
      targetComponent = broadcastId
    )

  val read: PartialFunction[MAVMessage, Unit] = {
    case `paramRequestList` =>
      logger.info("GroundControl> READ ParamRequestList")

      AutopilotParametersState.paramValues.foreach { paramValue =>
        logger.info("GroundControl> WRITE ParamValue")
        write(paramValue)
      }
    case paramRequestRead: GroundControl.ParamRequestRead
      if paramRequestRead.targetSystem == systemId &&
        paramRequestRead.targetComponent == localId =>
      logger.info("GroundControl> READ ParamRequestRead")

      for {
        paramValue <- AutopilotParametersState.paramValues
        if paramValue.paramId == paramRequestRead.paramId ||
          paramValue.paramIndex == paramRequestRead.paramIndex
      } {
        logger.info("GroundControl> WRITE ParamValue")
        write(paramValue)
      }
    case paramSet: GroundControl.ParamSet
      if paramSet.targetComponent == systemId &&
        paramSet.targetComponent == localId =>
      logger.info("GroundControl> READ ParamSet")

      for {
        index <- AutopilotParametersState.paramValues.indices
        paramValue = AutopilotParametersState.paramValues(index)
        if paramValue.paramId == paramSet.paramId
      } {
        val nextParamValue =
          paramValue.copy(
            paramValue = paramSet.paramValue,
            paramType = paramSet.paramType
          )

        AutopilotParametersState.paramValues(index) = nextParamValue
        logger.info("GroundControl> WRITE ParamValue")
        write(nextParamValue)
      }
  }
}
