package com.peterpotts.trax.mavlink.autopilot

import com.peterpotts.trax.mavlink.model.MAVEnums._
import com.peterpotts.trax.mavlink.model._

/**
  * @author Peter Potts
  */
case class GroundControlDownloadMissionIO(write: MAVMessage => Unit) extends AutopilotGroundControl {
  private val missionRequestList =
    GroundControl.MissionRequestList(
      targetSystem = systemId,
      targetComponent = localId,
      missionType = MAVMissionType.Mission
    )

  private val missionAck =
    GroundControl.MissionAck(
      targetSystem = systemId,
      targetComponent = localId,
      `type` = MAVMissionResult.Accepted,
      missionType = MAVMissionType.Mission
    )

  private def missionCount: Autopilot.MissionCount =
    Autopilot.MissionCount(
      targetSystem = systemId,
      targetComponent = localId,
      count = AutopilotMissionState.missionItemInts.length,
      missionType = MAVMissionType.Mission
    )

  private def getMissionItem(seq: Int): Autopilot.MissionItemInt = {
    val missionItem = AutopilotMissionState.missionItemInts(seq)

    Autopilot.MissionItemInt(
      targetSystem = systemId,
      targetComponent = remoteId,
      seq = seq,
      frame = missionItem.frame,
      command = missionItem.command,
      current = missionItem.current,
      autoContinue = missionItem.autoContinue,
      param1 = missionItem.param1,
      param2 = missionItem.param2,
      param3 = missionItem.param3,
      param4 = missionItem.param4,
      x = missionItem.x,
      y = missionItem.y,
      z = missionItem.z,
      missionType = missionItem.missionType
    )
  }

  val read: PartialFunction[MAVMessage, Unit] = {
    case missionRequest: GroundControl.MissionRequest
      if missionRequest.targetSystem == systemId &&
        missionRequest.targetComponent == localId &&
        missionRequest.missionType == MAVMissionType.Mission =>
      val seq = missionRequest.seq
      logger.info(s"GroundControl> READ MissionRequestInt $seq")
      logger.info(s"GroundControl> WRITE MissionItemInt $seq")
      write(getMissionItem(seq))
    case `missionRequestList` =>
      logger.info("GroundControl> READ MissionRequestList")
      logger.info("GroundControl> WRITE MissionCount")
      write(missionCount)
    case missionRequestInt: GroundControl.MissionRequestInt
      if missionRequestInt.targetSystem == systemId &&
        missionRequestInt.targetComponent == localId &&
        missionRequestInt.missionType == MAVMissionType.Mission =>
      val seq = missionRequestInt.seq
      logger.info(s"GroundControl> READ MissionRequestInt $seq")
      logger.info(s"GroundControl> WRITE MissionItemInt $seq")
      write(getMissionItem(seq))
    case `missionAck` =>
      logger.info("GroundControl> READ MissionAck")
  }
}
