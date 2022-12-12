package com.peterpotts.trax.mavlink.autopilot

import com.peterpotts.trax.mavlink.model.MAVEnums._
import com.peterpotts.trax.mavlink.model._

/**
  * @author Peter Potts
  */
case class GroundControlUploadMissionIO(write: MAVMessage => Unit) extends AutopilotGroundControl {
  private val missionClearAll =
    GroundControl.MissionClearAll(
      targetSystem = systemId,
      targetComponent = localId,
      missionType = MAVMissionType.Mission
    )

  private val missionAck =
    Autopilot.MissionAck(
      targetSystem = systemId,
      targetComponent = remoteId,
      `type` = MAVMissionResult.Accepted,
      missionType = MAVMissionType.Mission
    )

  private def getMissionRequestInt(seq: Int) =
    Autopilot.MissionRequestInt(
      targetSystem = GroundControl.systemId,
      targetComponent = GroundControl.componentId.value,
      seq = seq,
      missionType = MAVMissionType.Mission
    )

  private def setMissionItemInt(missionItemInt: GroundControl.MissionItemInt): Unit =
    AutopilotMissionState.missionItemInts(missionItemInt.seq) = missionItemInt

  val read: PartialFunction[MAVMessage, Unit] = {
    case `missionClearAll` =>
      logger.info(s"GroundControl> READ MissionClearAll")
      AutopilotMissionState.missionItemInts = new Array(0)
      logger.info("GroundControl> WRITE MissionAck")
      write(missionAck)
    case missionCount: GroundControl.MissionCount
      if missionCount.targetSystem == systemId &&
        missionCount.targetComponent == localId &&
        missionCount.missionType == MAVMissionType.Mission =>
      logger.info("GroundControl> READ MissionCount")
      AutopilotMissionState.missionItemInts = new Array(missionCount.count)
      logger.info("GroundControl> WRITE MissionRequestInt 0")
      write(getMissionRequestInt(0))
    case missionItemInt: GroundControl.MissionItemInt =>
      logger.info(s"GroundControl> READ MissionItemInt ${missionItemInt.seq}")
      setMissionItemInt(missionItemInt)
      logger.info(s"GroundControl> UPLOAD $missionItemInt")
      val nextSeq = missionItemInt.seq + 1

      if (nextSeq < AutopilotMissionState.missionItemInts.length) {
        logger.info(s"GroundControl> WRITE MissionRequestInt $nextSeq")
        write(getMissionRequestInt(nextSeq))
      } else {
        logger.info("GroundControl> WRITE MissionAck")
        write(missionAck)
      }
  }
}
