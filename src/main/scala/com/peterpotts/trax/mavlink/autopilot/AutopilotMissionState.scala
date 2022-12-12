package com.peterpotts.trax.mavlink.autopilot

import com.peterpotts.trax.mavlink.model.GroundControl

/**
  * @author Peter Potts
  */
object AutopilotMissionState {
  /**
    * {{{
    * targetSystem    System ID
    * targetComponent Component ID
    * seq             Waypoint ID (sequence number). Starts at zero.
    *                 Increases monotonically for each waypoint,
    *                 no gaps in the sequence (0,1,2,3,4).
    * frame           The coordinate system of the waypoint.
    *                 See MAV_FRAME in mavlink_types.h
    * command         The scheduled action for the waypoint.
    *                 See MAV_CMD in common.xml MAVLink specs
    * current         false:0, true:1
    * autoContinue    autocontinue to next wp
    * param1          PARAM1, see MAV_CMD enum
    * param2          PARAM2, see MAV_CMD enum
    * param3          PARAM3, see MAV_CMD enum
    * param4          PARAM4, see MAV_CMD enum
    * x               PARAM5 / local: x position in meters * 1e4,
    *                          global: latitude in degrees * 10^^7
    * y               PARAM6 / y position: local: x position in meters * 1e4,
    *                                      global: longitude in degrees *10^^7
    * z               PARAM7 / z position: global: altitude in meters
    *                                      (relative or absolute, depending on frame).
    * missionType     Mission type, see MAV_MISSION_TYPE
    * }}}
    */
  var missionItemInts: Array[GroundControl.MissionItemInt] = Array.empty
}
