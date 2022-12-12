package com.peterpotts.trax.mavlink.autopilot

import com.peterpotts.trax.mavlink.model.Autopilot
import com.peterpotts.trax.mavlink.model.MAVEnums.MAVParamType

/**
  * @author Peter Potts
  */
object AutopilotParametersState {
  /**
    * {{{
    * paramId    Onboard parameter id,
    *            terminated by NULL
    *            if the length is less than 16 human-readable chars and
    *            WITHOUT null termination (NULL) byte
    *            if the length is exactly 16 chars -
    *            applications have to provide 16+1 bytes storage if the ID is stored as string
    * paramValue Onboard parameter value
    * paramType  Onboard parameter type: see the MAV_PARAM_TYPE enum for supported data types.
    * paramCount Total number of onboard parameters
    * paramIndex Index of this onboard parameter
    * }}}
    */
  var paramValues: Array[Autopilot.ParamValue] =
    Array(
      Autopilot.ParamValue(
        paramId = "Test",
        paramValue = 1.23f,
        paramType = MAVParamType.Real32,
        paramCount = 1,
        paramIndex = 0
      )
    )
}
