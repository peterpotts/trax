package com.peterpotts.trax.mavlink.autopilot

import com.peterpotts.trax.mavlink.io.MAVLinkReader
import com.peterpotts.trax.mavlink.model.MAVMessage

/**
  * @author Peter Potts
  */
case class GroundControlReader(write: MAVMessage => Unit) extends MAVLinkReader {
  private val heartbeatIO: GroundControlHeartbeatIO = GroundControlHeartbeatIO(write)
  private val pingIO: GroundControlPingIO = GroundControlPingIO(write)
  private val stateVectorIO: GroundControlStateIO = GroundControlStateIO(write)
  private val timeIO: GroundControlTimeIO = GroundControlTimeIO(write)
  private val paramIO: GroundControlParamIO = GroundControlParamIO(write)
  private val versionIO: GroundControlVersionIO = GroundControlVersionIO(write)
  private val capabilitiesIO: GroundControlCapabilitiesIO = GroundControlCapabilitiesIO(write)
  private val manualIO: GroundControlManualIO = GroundControlManualIO(write)
  private val armIO: GroundControlArmIO = GroundControlArmIO(write)
  private val downloadMissionIO: GroundControlDownloadMissionIO = GroundControlDownloadMissionIO(write)
  private val uploadMissionIO: GroundControlUploadMissionIO = GroundControlUploadMissionIO(write)

  private val default: PartialFunction[MAVMessage, Unit] = {
    case message => logger.info(s"GroundControl> READ *** $message ***")
  }

  private val partialFunction =
    heartbeatIO.read orElse
      pingIO.read orElse
      stateVectorIO.read orElse
      timeIO.read orElse
      paramIO.read orElse
      versionIO.read orElse
      capabilitiesIO.read orElse
      manualIO.read orElse
      armIO.read orElse
      downloadMissionIO.read orElse
      uploadMissionIO.read orElse
      default

  def tick(sequenceNumber: Int): Unit = {
    heartbeatIO.tick(sequenceNumber)
    pingIO.tick(sequenceNumber)
    stateVectorIO.tick(sequenceNumber)
  }

  def read(message: MAVMessage): Unit = partialFunction(message)
}
