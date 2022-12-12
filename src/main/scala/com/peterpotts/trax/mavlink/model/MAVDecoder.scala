package com.peterpotts.trax.mavlink.model

import com.peterpotts.trax.mavlink.model.MAVEnums._
import me.drton.jmavlib.mavlink.MAVLinkMessage

/**
  * @author Peter Potts
  */
object MAVDecoder {
  def apply(message: MAVLinkMessage): MAVMessage = {
    val messages = Component(message)

    message.msgID match {
      case 0 =>
        messages.Heartbeat(
          `type` = MAVType(message.getInt("type")),
          autopilot = MAVAutopilot(message.getInt("autopilot")),
          baseMode = MAVModeFlag.bitmask(message.getInt("base_mode")),
          customMode = message.getInt("custom_mode"),
          systemStatus = MAVState(message.getInt("system_status"))
        )
      case 1 =>
        messages.SysStatus(
          onboardControlSensorsPresent = MAVSysStatusSensor.bitmask(message.getInt("onboard_control_sensors_present")),
          onboardControlSensorsEnabled = MAVSysStatusSensor.bitmask(message.getInt("onboard_control_sensors_enabled")),
          onboardControlSensorsHealth = MAVSysStatusSensor.bitmask(message.getInt("onboard_control_sensors_health")),
          load = message.getInt("load"),
          voltageBattery = message.getInt("voltage_battery"),
          currentBattery = message.getInt("current_battery"),
          batteryRemaining = message.getInt("battery_remaining"),
          dropRateComm = message.getInt("drop_rate_comm"),
          errorsComm = message.getInt("errors_comm"),
          errorsCount1 = message.getInt("errors_count1"),
          errorsCount2 = message.getInt("errors_count2"),
          errorsCount3 = message.getInt("errors_count3"),
          errorsCount4 = message.getInt("errors_count4")
        )
      case 2 =>
        messages.SystemTime(
          timeUnixUSec = message.getLong("time_unix_usec"),
          timeBootMS = message.getInt("time_boot_ms")
        )
      case 4 =>
        messages.Ping(
          timeUSec = message.getLong("time_usec"),
          seq = message.getInt("seq"),
          targetSystem = message.getInt("target_system"),
          targetComponent = message.getInt("target_component")
        )
      case 5 =>
        messages.ChangeOperatorControl(
          targetSystem = message.getInt("target_system"),
          controlRequest = message.getInt("control_request"),
          version = message.getInt("version"),
          passkey = message.getString("passkey")
        )
      case 6 =>
        messages.ChangeOperatorControlAck(
          gCSSystemId = message.getInt("gcs_system_id"),
          controlRequest = message.getInt("control_request"),
          ack = message.getInt("ack")
        )
      case 7 =>
        messages.AuthKey(
          key = message.getString("key")
        )
      case 11 =>
        messages.SetMode(
          targetSystem = message.getInt("target_system"),
          baseMode = MAVMode(message.getInt("base_mode")),
          customMode = message.getInt("custom_mode")
        )
      case 20 =>
        messages.ParamRequestRead(
          targetSystem = message.getInt("target_system"),
          targetComponent = message.getInt("target_component"),
          paramId = message.getString("param_id"),
          paramIndex = message.getInt("param_index")
        )
      case 21 =>
        messages.ParamRequestList(
          targetSystem = message.getInt("target_system"),
          targetComponent = message.getInt("target_component")
        )
      case 22 =>
        messages.ParamValue(
          paramId = message.getString("param_id"),
          paramValue = message.getFloat("param_value"),
          paramType = MAVParamType(message.getInt("param_type")),
          paramCount = message.getInt("param_count"),
          paramIndex = message.getInt("param_index")
        )
      case 23 =>
        messages.ParamSet(
          targetSystem = message.getInt("target_system"),
          targetComponent = message.getInt("target_component"),
          paramId = message.getString("param_id"),
          paramValue = message.getFloat("param_value"),
          paramType = MAVParamType(message.getInt("param_type"))
        )
      case 24 =>
        messages.GPSRawInt(
          timeUSec = message.getLong("time_usec"),
          fixType = GPSFixType(message.getInt("fix_type")),
          lat = message.getInt("lat"),
          lon = message.getInt("lon"),
          alt = message.getInt("alt"),
          epH = message.getInt("eph"),
          epV = message.getInt("epv"),
          vel = message.getInt("vel"),
          cOG = message.getInt("cog"),
          satellitesVisible = message.getInt("satellites_visible"),
          altEllipsoid = message.getInt("alt_ellipsoid"),
          hAcc = message.getInt("h_acc"),
          vAcc = message.getInt("v_acc"),
          velAcc = message.getInt("vel_acc"),
          hdgAcc = message.getInt("hdg_acc")
        )
      case 25 =>
        messages.GPSStatus(
          satellitesVisible = message.getInt("satellites_visible"),
          satellitePRN = message.get("satellite_prn").asInstanceOf[Array[AnyRef]].map(_.asInstanceOf[java.lang.Integer].toInt).toIndexedSeq,
          satelliteUsed = message.get("satellite_used").asInstanceOf[Array[AnyRef]].map(_.asInstanceOf[java.lang.Integer].toInt).toIndexedSeq,
          satelliteElevation = message.get("satellite_elevation").asInstanceOf[Array[AnyRef]].map(_.asInstanceOf[java.lang.Integer].toInt).toIndexedSeq,
          satelliteAzimuth = message.get("satellite_azimuth").asInstanceOf[Array[AnyRef]].map(_.asInstanceOf[java.lang.Integer].toInt).toIndexedSeq,
          satelliteSNR = message.get("satellite_snr").asInstanceOf[Array[AnyRef]].map(_.asInstanceOf[java.lang.Integer].toInt).toIndexedSeq
        )
      case 26 =>
        messages.ScaledIMU(
          timeBootMS = message.getInt("time_boot_ms"),
          xAcc = message.getInt("xacc"),
          yAcc = message.getInt("yacc"),
          zAcc = message.getInt("zacc"),
          xGyro = message.getInt("xgyro"),
          yGyro = message.getInt("ygyro"),
          zGyro = message.getInt("zgyro"),
          xMag = message.getInt("xmag"),
          yMag = message.getInt("ymag"),
          zMag = message.getInt("zmag")
        )
      case 27 =>
        messages.RawIMU(
          timeUSec = message.getLong("time_usec"),
          xAcc = message.getInt("xacc"),
          yAcc = message.getInt("yacc"),
          zAcc = message.getInt("zacc"),
          xGyro = message.getInt("xgyro"),
          yGyro = message.getInt("ygyro"),
          zGyro = message.getInt("zgyro"),
          xMag = message.getInt("xmag"),
          yMag = message.getInt("ymag"),
          zMag = message.getInt("zmag")
        )
      case 28 =>
        messages.RawPressure(
          timeUSec = message.getLong("time_usec"),
          pressAbs = message.getInt("press_abs"),
          pressDiff1 = message.getInt("press_diff1"),
          pressDiff2 = message.getInt("press_diff2"),
          temperature = message.getInt("temperature")
        )
      case 29 =>
        messages.ScaledPressure(
          timeBootMS = message.getInt("time_boot_ms"),
          pressAbs = message.getFloat("press_abs"),
          pressDiff = message.getFloat("press_diff"),
          temperature = message.getInt("temperature")
        )
      case 30 =>
        messages.Attitude(
          timeBootMS = message.getInt("time_boot_ms"),
          roll = message.getFloat("roll"),
          pitch = message.getFloat("pitch"),
          yaw = message.getFloat("yaw"),
          rollSpeed = message.getFloat("rollspeed"),
          pitchSpeed = message.getFloat("pitchspeed"),
          yawSpeed = message.getFloat("yawspeed")
        )
      case 31 =>
        messages.AttitudeQuaternion(
          timeBootMS = message.getInt("time_boot_ms"),
          q1 = message.getFloat("q1"),
          q2 = message.getFloat("q2"),
          q3 = message.getFloat("q3"),
          q4 = message.getFloat("q4"),
          rollSpeed = message.getFloat("rollspeed"),
          pitchSpeed = message.getFloat("pitchspeed"),
          yawSpeed = message.getFloat("yawspeed")
        )
      case 32 =>
        messages.LocalPositionNED(
          timeBootMS = message.getInt("time_boot_ms"),
          x = message.getFloat("x"),
          y = message.getFloat("y"),
          z = message.getFloat("z"),
          vX = message.getFloat("vx"),
          vY = message.getFloat("vy"),
          vZ = message.getFloat("vz")
        )
      case 33 =>
        messages.GlobalPositionInt(
          timeBootMS = message.getInt("time_boot_ms"),
          lat = message.getInt("lat"),
          lon = message.getInt("lon"),
          alt = message.getInt("alt"),
          relativeAlt = message.getInt("relative_alt"),
          vX = message.getInt("vx"),
          vY = message.getInt("vy"),
          vZ = message.getInt("vz"),
          hdg = message.getInt("hdg")
        )
      case 34 =>
        messages.RCChannelsScaled(
          timeBootMS = message.getInt("time_boot_ms"),
          port = message.getInt("port"),
          chan1Scaled = message.getInt("chan1_scaled"),
          chan2Scaled = message.getInt("chan2_scaled"),
          chan3Scaled = message.getInt("chan3_scaled"),
          chan4Scaled = message.getInt("chan4_scaled"),
          chan5Scaled = message.getInt("chan5_scaled"),
          chan6Scaled = message.getInt("chan6_scaled"),
          chan7Scaled = message.getInt("chan7_scaled"),
          chan8Scaled = message.getInt("chan8_scaled"),
          rSSI = message.getInt("rssi")
        )
      case 35 =>
        messages.RCChannelsRaw(
          timeBootMS = message.getInt("time_boot_ms"),
          port = message.getInt("port"),
          chan1Raw = message.getInt("chan1_raw"),
          chan2Raw = message.getInt("chan2_raw"),
          chan3Raw = message.getInt("chan3_raw"),
          chan4Raw = message.getInt("chan4_raw"),
          chan5Raw = message.getInt("chan5_raw"),
          chan6Raw = message.getInt("chan6_raw"),
          chan7Raw = message.getInt("chan7_raw"),
          chan8Raw = message.getInt("chan8_raw"),
          rSSI = message.getInt("rssi")
        )
      case 36 =>
        messages.ServoOutputRaw(
          timeUSec = message.getInt("time_usec"),
          port = message.getInt("port"),
          servo1Raw = message.getInt("servo1_raw"),
          servo2Raw = message.getInt("servo2_raw"),
          servo3Raw = message.getInt("servo3_raw"),
          servo4Raw = message.getInt("servo4_raw"),
          servo5Raw = message.getInt("servo5_raw"),
          servo6Raw = message.getInt("servo6_raw"),
          servo7Raw = message.getInt("servo7_raw"),
          servo8Raw = message.getInt("servo8_raw"),
          servo9Raw = message.getInt("servo9_raw"),
          servo10Raw = message.getInt("servo10_raw"),
          servo11Raw = message.getInt("servo11_raw"),
          servo12Raw = message.getInt("servo12_raw"),
          servo13Raw = message.getInt("servo13_raw"),
          servo14Raw = message.getInt("servo14_raw"),
          servo15Raw = message.getInt("servo15_raw"),
          servo16Raw = message.getInt("servo16_raw")
        )
      case 37 =>
        messages.MissionRequestPartialList(
          targetSystem = message.getInt("target_system"),
          targetComponent = message.getInt("target_component"),
          startIndex = message.getInt("start_index"),
          endIndex = message.getInt("end_index"),
          missionType = MAVMissionType(message.getInt("mission_type"))
        )
      case 38 =>
        messages.MissionWritePartialList(
          targetSystem = message.getInt("target_system"),
          targetComponent = message.getInt("target_component"),
          startIndex = message.getInt("start_index"),
          endIndex = message.getInt("end_index"),
          missionType = MAVMissionType(message.getInt("mission_type"))
        )
      case 39 =>
        messages.MissionItem(
          targetSystem = message.getInt("target_system"),
          targetComponent = message.getInt("target_component"),
          seq = message.getInt("seq"),
          frame = MAVFrame(message.getInt("frame")),
          command = MAVCmd(message.getInt("command")),
          current = message.getInt("current"),
          autoContinue = message.getInt("autocontinue"),
          param1 = message.getFloat("param1"),
          param2 = message.getFloat("param2"),
          param3 = message.getFloat("param3"),
          param4 = message.getFloat("param4"),
          x = message.getFloat("x"),
          y = message.getFloat("y"),
          z = message.getFloat("z"),
          missionType = MAVMissionType(message.getInt("mission_type"))
        )
      case 40 =>
        messages.MissionRequest(
          targetSystem = message.getInt("target_system"),
          targetComponent = message.getInt("target_component"),
          seq = message.getInt("seq"),
          missionType = MAVMissionType(message.getInt("mission_type"))
        )
      case 41 =>
        messages.MissionSetCurrent(
          targetSystem = message.getInt("target_system"),
          targetComponent = message.getInt("target_component"),
          seq = message.getInt("seq")
        )
      case 42 =>
        messages.MissionCurrent(
          seq = message.getInt("seq")
        )
      case 43 =>
        messages.MissionRequestList(
          targetSystem = message.getInt("target_system"),
          targetComponent = message.getInt("target_component"),
          missionType = MAVMissionType(message.getInt("mission_type"))
        )
      case 44 =>
        messages.MissionCount(
          targetSystem = message.getInt("target_system"),
          targetComponent = message.getInt("target_component"),
          count = message.getInt("count"),
          missionType = MAVMissionType(message.getInt("mission_type"))
        )
      case 45 =>
        messages.MissionClearAll(
          targetSystem = message.getInt("target_system"),
          targetComponent = message.getInt("target_component"),
          missionType = MAVMissionType(message.getInt("mission_type"))
        )
      case 46 =>
        messages.MissionItemReached(
          seq = message.getInt("seq")
        )
      case 47 =>
        messages.MissionAck(
          targetSystem = message.getInt("target_system"),
          targetComponent = message.getInt("target_component"),
          `type` = MAVMissionResult(message.getInt("type")),
          missionType = MAVMissionType(message.getInt("mission_type"))
        )
      case 48 =>
        messages.SetGPSGlobalOrigin(
          targetSystem = message.getInt("target_system"),
          latitude = message.getInt("latitude"),
          longitude = message.getInt("longitude"),
          altitude = message.getInt("altitude"),
          timeUSec = message.getLong("time_usec")
        )
      case 49 =>
        messages.GPSGlobalOrigin(
          latitude = message.getInt("latitude"),
          longitude = message.getInt("longitude"),
          altitude = message.getInt("altitude"),
          timeUSec = message.getLong("time_usec")
        )
      case 50 =>
        messages.ParamMapRC(
          targetSystem = message.getInt("target_system"),
          targetComponent = message.getInt("target_component"),
          paramId = message.getString("param_id"),
          paramIndex = message.getInt("param_index"),
          parameterRCChannelIndex = message.getInt("parameter_rc_channel_index"),
          paramValue0 = message.getFloat("param_value0"),
          scale = message.getFloat("scale"),
          paramValueMin = message.getFloat("param_value_min"),
          paramValueMax = message.getFloat("param_value_max")
        )
      case 51 =>
        messages.MissionRequestInt(
          targetSystem = message.getInt("target_system"),
          targetComponent = message.getInt("target_component"),
          seq = message.getInt("seq"),
          missionType = MAVMissionType(message.getInt("mission_type"))
        )
      case 54 =>
        messages.SafetySetAllowedArea(
          targetSystem = message.getInt("target_system"),
          targetComponent = message.getInt("target_component"),
          frame = MAVFrame(message.getInt("frame")),
          p1X = message.getFloat("p1x"),
          p1Y = message.getFloat("p1y"),
          p1Z = message.getFloat("p1z"),
          p2X = message.getFloat("p2x"),
          p2Y = message.getFloat("p2y"),
          p2Z = message.getFloat("p2z")
        )
      case 55 =>
        messages.SafetyAllowedArea(
          frame = MAVFrame(message.getInt("frame")),
          p1X = message.getFloat("p1x"),
          p1Y = message.getFloat("p1y"),
          p1Z = message.getFloat("p1z"),
          p2X = message.getFloat("p2x"),
          p2Y = message.getFloat("p2y"),
          p2Z = message.getFloat("p2z")
        )
      case 61 =>
        messages.AttitudeQuaternionCov(
          timeUSec = message.getLong("time_usec"),
          q = message.get("q").asInstanceOf[Array[AnyRef]].map(_.asInstanceOf[java.lang.Float].toFloat).toIndexedSeq,
          rollSpeed = message.getFloat("rollspeed"),
          pitchSpeed = message.getFloat("pitchspeed"),
          yawSpeed = message.getFloat("yawspeed"),
          covariance = message.get("covariance").asInstanceOf[Array[AnyRef]].map(_.asInstanceOf[java.lang.Float].toFloat).toIndexedSeq
        )
      case 62 =>
        messages.NavControllerOutput(
          navRoll = message.getFloat("nav_roll"),
          navPitch = message.getFloat("nav_pitch"),
          navBearing = message.getInt("nav_bearing"),
          targetBearing = message.getInt("target_bearing"),
          wpDist = message.getInt("wp_dist"),
          altError = message.getFloat("alt_error"),
          aSPDError = message.getFloat("aspd_error"),
          xTrackError = message.getFloat("xtrack_error")
        )
      case 63 =>
        messages.GlobalPositionIntCov(
          timeUSec = message.getLong("time_usec"),
          estimatorType = MAVEstimatorType(message.getInt("estimator_type")),
          lat = message.getInt("lat"),
          lon = message.getInt("lon"),
          alt = message.getInt("alt"),
          relativeAlt = message.getInt("relative_alt"),
          vX = message.getFloat("vx"),
          vY = message.getFloat("vy"),
          vZ = message.getFloat("vz"),
          covariance = message.get("covariance").asInstanceOf[Array[AnyRef]].map(_.asInstanceOf[java.lang.Float].toFloat).toIndexedSeq
        )
      case 64 =>
        messages.LocalPositionNEDCov(
          timeUSec = message.getLong("time_usec"),
          estimatorType = MAVEstimatorType(message.getInt("estimator_type")),
          x = message.getFloat("x"),
          y = message.getFloat("y"),
          z = message.getFloat("z"),
          vX = message.getFloat("vx"),
          vY = message.getFloat("vy"),
          vZ = message.getFloat("vz"),
          aX = message.getFloat("ax"),
          aY = message.getFloat("ay"),
          aZ = message.getFloat("az"),
          covariance = message.get("covariance").asInstanceOf[Array[AnyRef]].map(_.asInstanceOf[java.lang.Float].toFloat).toIndexedSeq
        )
      case 65 =>
        messages.RCChannels(
          timeBootMS = message.getInt("time_boot_ms"),
          chanCount = message.getInt("chancount"),
          chan1Raw = message.getInt("chan1_raw"),
          chan2Raw = message.getInt("chan2_raw"),
          chan3Raw = message.getInt("chan3_raw"),
          chan4Raw = message.getInt("chan4_raw"),
          chan5Raw = message.getInt("chan5_raw"),
          chan6Raw = message.getInt("chan6_raw"),
          chan7Raw = message.getInt("chan7_raw"),
          chan8Raw = message.getInt("chan8_raw"),
          chan9Raw = message.getInt("chan9_raw"),
          chan10Raw = message.getInt("chan10_raw"),
          chan11Raw = message.getInt("chan11_raw"),
          chan12Raw = message.getInt("chan12_raw"),
          chan13Raw = message.getInt("chan13_raw"),
          chan14Raw = message.getInt("chan14_raw"),
          chan15Raw = message.getInt("chan15_raw"),
          chan16Raw = message.getInt("chan16_raw"),
          chan17Raw = message.getInt("chan17_raw"),
          chan18Raw = message.getInt("chan18_raw"),
          rSSI = message.getInt("rssi")
        )
      case 66 =>
        messages.RequestDataStream(
          targetSystem = message.getInt("target_system"),
          targetComponent = message.getInt("target_component"),
          reqStreamId = message.getInt("req_stream_id"),
          reqMessageRate = message.getInt("req_message_rate"),
          startStop = message.getInt("start_stop")
        )
      case 67 =>
        messages.DataStream(
          streamId = message.getInt("stream_id"),
          messageRate = message.getInt("message_rate"),
          onOff = message.getInt("on_off")
        )
      case 69 =>
        messages.ManualControl(
          target = message.getInt("target"),
          x = message.getInt("x"),
          y = message.getInt("y"),
          z = message.getInt("z"),
          r = message.getInt("r"),
          buttons = message.getInt("buttons")
        )
      case 70 =>
        messages.RCChannelsOverride(
          targetSystem = message.getInt("target_system"),
          targetComponent = message.getInt("target_component"),
          chan1Raw = message.getInt("chan1_raw"),
          chan2Raw = message.getInt("chan2_raw"),
          chan3Raw = message.getInt("chan3_raw"),
          chan4Raw = message.getInt("chan4_raw"),
          chan5Raw = message.getInt("chan5_raw"),
          chan6Raw = message.getInt("chan6_raw"),
          chan7Raw = message.getInt("chan7_raw"),
          chan8Raw = message.getInt("chan8_raw")
        )
      case 73 =>
        messages.MissionItemInt(
          targetSystem = message.getInt("target_system"),
          targetComponent = message.getInt("target_component"),
          seq = message.getInt("seq"),
          frame = MAVFrame(message.getInt("frame")),
          command = MAVCmd(message.getInt("command")),
          current = message.getInt("current"),
          autoContinue = message.getInt("autocontinue"),
          param1 = message.getFloat("param1"),
          param2 = message.getFloat("param2"),
          param3 = message.getFloat("param3"),
          param4 = message.getFloat("param4"),
          x = message.getInt("x"),
          y = message.getInt("y"),
          z = message.getFloat("z"),
          missionType = MAVMissionType(message.getInt("mission_type"))
        )
      case 74 =>
        messages.VFRHUD(
          airspeed = message.getFloat("airspeed"),
          groundSpeed = message.getFloat("groundspeed"),
          heading = message.getInt("heading"),
          throttle = message.getInt("throttle"),
          alt = message.getFloat("alt"),
          climb = message.getFloat("climb")
        )
      case 75 =>
        messages.CommandInt(
          targetSystem = message.getInt("target_system"),
          targetComponent = message.getInt("target_component"),
          frame = MAVFrame(message.getInt("frame")),
          command = MAVCmd(message.getInt("command")),
          current = message.getInt("current"),
          autoContinue = message.getInt("autocontinue"),
          param1 = message.getFloat("param1"),
          param2 = message.getFloat("param2"),
          param3 = message.getFloat("param3"),
          param4 = message.getFloat("param4"),
          x = message.getInt("x"),
          y = message.getInt("y"),
          z = message.getFloat("z")
        )
      case 76 =>
        messages.CommandLong(
          targetSystem = message.getInt("target_system"),
          targetComponent = message.getInt("target_component"),
          command = MAVCmd(message.getInt("command")),
          confirmation = message.getInt("confirmation"),
          param1 = message.getFloat("param1"),
          param2 = message.getFloat("param2"),
          param3 = message.getFloat("param3"),
          param4 = message.getFloat("param4"),
          param5 = message.getFloat("param5"),
          param6 = message.getFloat("param6"),
          param7 = message.getFloat("param7")
        )
      case 77 =>
        messages.CommandAck(
          command = MAVCmd(message.getInt("command")),
          result = MAVResult(message.getInt("result")),
          progress = message.getInt("progress"),
          resultParam2 = message.getInt("result_param2"),
          targetSystem = message.getInt("target_system"),
          targetComponent = message.getInt("target_component")
        )
      case 81 =>
        messages.ManualSetpoint(
          timeBootMS = message.getInt("time_boot_ms"),
          roll = message.getFloat("roll"),
          pitch = message.getFloat("pitch"),
          yaw = message.getFloat("yaw"),
          thrust = message.getFloat("thrust"),
          modeSwitch = message.getInt("mode_switch"),
          manualOverrideSwitch = message.getInt("manual_override_switch")
        )
      case 82 =>
        messages.SetAttitudeTarget(
          timeBootMS = message.getInt("time_boot_ms"),
          targetSystem = message.getInt("target_system"),
          targetComponent = message.getInt("target_component"),
          typeMask = message.getInt("type_mask"),
          q = message.get("q").asInstanceOf[Array[AnyRef]].map(_.asInstanceOf[java.lang.Float].toFloat).toIndexedSeq,
          bodyRollRate = message.getFloat("body_roll_rate"),
          bodyPitchRate = message.getFloat("body_pitch_rate"),
          bodyYawRate = message.getFloat("body_yaw_rate"),
          thrust = message.getFloat("thrust")
        )
      case 83 =>
        messages.AttitudeTarget(
          timeBootMS = message.getInt("time_boot_ms"),
          typeMask = message.getInt("type_mask"),
          q = message.get("q").asInstanceOf[Array[AnyRef]].map(_.asInstanceOf[java.lang.Float].toFloat).toIndexedSeq,
          bodyRollRate = message.getFloat("body_roll_rate"),
          bodyPitchRate = message.getFloat("body_pitch_rate"),
          bodyYawRate = message.getFloat("body_yaw_rate"),
          thrust = message.getFloat("thrust")
        )
      case 84 =>
        messages.SetPositionTargetLocalNED(
          timeBootMS = message.getInt("time_boot_ms"),
          targetSystem = message.getInt("target_system"),
          targetComponent = message.getInt("target_component"),
          coordinateFrame = MAVFrame(message.getInt("coordinate_frame")),
          typeMask = message.getInt("type_mask"),
          x = message.getFloat("x"),
          y = message.getFloat("y"),
          z = message.getFloat("z"),
          vX = message.getFloat("vx"),
          vY = message.getFloat("vy"),
          vZ = message.getFloat("vz"),
          aFX = message.getFloat("afx"),
          aFY = message.getFloat("afy"),
          aFZ = message.getFloat("afz"),
          yaw = message.getFloat("yaw"),
          yawRate = message.getFloat("yaw_rate")
        )
      case 85 =>
        messages.PositionTargetLocalNED(
          timeBootMS = message.getInt("time_boot_ms"),
          coordinateFrame = MAVFrame(message.getInt("coordinate_frame")),
          typeMask = message.getInt("type_mask"),
          x = message.getFloat("x"),
          y = message.getFloat("y"),
          z = message.getFloat("z"),
          vX = message.getFloat("vx"),
          vY = message.getFloat("vy"),
          vZ = message.getFloat("vz"),
          aFX = message.getFloat("afx"),
          aFY = message.getFloat("afy"),
          aFZ = message.getFloat("afz"),
          yaw = message.getFloat("yaw"),
          yawRate = message.getFloat("yaw_rate")
        )
      case 86 =>
        messages.SetPositionTargetGlobalInt(
          timeBootMS = message.getInt("time_boot_ms"),
          targetSystem = message.getInt("target_system"),
          targetComponent = message.getInt("target_component"),
          coordinateFrame = MAVFrame(message.getInt("coordinate_frame")),
          typeMask = message.getInt("type_mask"),
          latInt = message.getInt("lat_int"),
          lonInt = message.getInt("lon_int"),
          alt = message.getFloat("alt"),
          vX = message.getFloat("vx"),
          vY = message.getFloat("vy"),
          vZ = message.getFloat("vz"),
          aFX = message.getFloat("afx"),
          aFY = message.getFloat("afy"),
          aFZ = message.getFloat("afz"),
          yaw = message.getFloat("yaw"),
          yawRate = message.getFloat("yaw_rate")
        )
      case 87 =>
        messages.PositionTargetGlobalInt(
          timeBootMS = message.getInt("time_boot_ms"),
          coordinateFrame = MAVFrame(message.getInt("coordinate_frame")),
          typeMask = message.getInt("type_mask"),
          latInt = message.getInt("lat_int"),
          lonInt = message.getInt("lon_int"),
          alt = message.getFloat("alt"),
          vX = message.getFloat("vx"),
          vY = message.getFloat("vy"),
          vZ = message.getFloat("vz"),
          aFX = message.getFloat("afx"),
          aFY = message.getFloat("afy"),
          aFZ = message.getFloat("afz"),
          yaw = message.getFloat("yaw"),
          yawRate = message.getFloat("yaw_rate")
        )
      case 89 =>
        messages.LocalPositionNEDSystemGlobalOffset(
          timeBootMS = message.getInt("time_boot_ms"),
          x = message.getFloat("x"),
          y = message.getFloat("y"),
          z = message.getFloat("z"),
          roll = message.getFloat("roll"),
          pitch = message.getFloat("pitch"),
          yaw = message.getFloat("yaw")
        )
      case 90 =>
        messages.HILState(
          timeUSec = message.getLong("time_usec"),
          roll = message.getFloat("roll"),
          pitch = message.getFloat("pitch"),
          yaw = message.getFloat("yaw"),
          rollSpeed = message.getFloat("rollspeed"),
          pitchSpeed = message.getFloat("pitchspeed"),
          yawSpeed = message.getFloat("yawspeed"),
          lat = message.getInt("lat"),
          lon = message.getInt("lon"),
          alt = message.getInt("alt"),
          vX = message.getInt("vx"),
          vY = message.getInt("vy"),
          vZ = message.getInt("vz"),
          xAcc = message.getInt("xacc"),
          yAcc = message.getInt("yacc"),
          zAcc = message.getInt("zacc")
        )
      case 91 =>
        messages.HILControls(
          timeUSec = message.getLong("time_usec"),
          rollAilerons = message.getFloat("roll_ailerons"),
          pitchElevator = message.getFloat("pitch_elevator"),
          yawRudder = message.getFloat("yaw_rudder"),
          throttle = message.getFloat("throttle"),
          aux1 = message.getFloat("aux1"),
          aux2 = message.getFloat("aux2"),
          aux3 = message.getFloat("aux3"),
          aux4 = message.getFloat("aux4"),
          mode = MAVMode(message.getInt("mode")),
          navMode = message.getInt("nav_mode")
        )
      case 92 =>
        messages.HILRCInputsRaw(
          timeUSec = message.getLong("time_usec"),
          chan1Raw = message.getInt("chan1_raw"),
          chan2Raw = message.getInt("chan2_raw"),
          chan3Raw = message.getInt("chan3_raw"),
          chan4Raw = message.getInt("chan4_raw"),
          chan5Raw = message.getInt("chan5_raw"),
          chan6Raw = message.getInt("chan6_raw"),
          chan7Raw = message.getInt("chan7_raw"),
          chan8Raw = message.getInt("chan8_raw"),
          chan9Raw = message.getInt("chan9_raw"),
          chan10Raw = message.getInt("chan10_raw"),
          chan11Raw = message.getInt("chan11_raw"),
          chan12Raw = message.getInt("chan12_raw"),
          rSSI = message.getInt("rssi")
        )
      case 93 =>
        messages.HILActuatorControls(
          timeUSec = message.getLong("time_usec"),
          controls = message.get("controls").asInstanceOf[Array[AnyRef]].map(_.asInstanceOf[java.lang.Float].toFloat).toIndexedSeq,
          mode = MAVMode(message.getInt("mode")),
          flags = message.getLong("flags")
        )
      case 100 =>
        messages.OpticalFlow(
          timeUSec = message.getLong("time_usec"),
          sensorId = message.getInt("sensor_id"),
          flowX = message.getInt("flow_x"),
          flowY = message.getInt("flow_y"),
          flowCompMX = message.getFloat("flow_comp_m_x"),
          flowCompMY = message.getFloat("flow_comp_m_y"),
          quality = message.getInt("quality"),
          groundDistance = message.getFloat("ground_distance"),
          flowRateX = message.getFloat("flow_rate_x"),
          flowRateY = message.getFloat("flow_rate_y")
        )
      case 101 =>
        messages.GlobalVisionPositionEstimate(
          uSec = message.getLong("usec"),
          x = message.getFloat("x"),
          y = message.getFloat("y"),
          z = message.getFloat("z"),
          roll = message.getFloat("roll"),
          pitch = message.getFloat("pitch"),
          yaw = message.getFloat("yaw")
        )
      case 102 =>
        messages.VisionPositionEstimate(
          uSec = message.getLong("usec"),
          x = message.getFloat("x"),
          y = message.getFloat("y"),
          z = message.getFloat("z"),
          roll = message.getFloat("roll"),
          pitch = message.getFloat("pitch"),
          yaw = message.getFloat("yaw")
        )
      case 103 =>
        messages.VisionSpeedEstimate(
          uSec = message.getLong("usec"),
          x = message.getFloat("x"),
          y = message.getFloat("y"),
          z = message.getFloat("z")
        )
      case 104 =>
        messages.ViconPositionEstimate(
          uSec = message.getLong("usec"),
          x = message.getFloat("x"),
          y = message.getFloat("y"),
          z = message.getFloat("z"),
          roll = message.getFloat("roll"),
          pitch = message.getFloat("pitch"),
          yaw = message.getFloat("yaw")
        )
      case 105 =>
        messages.HighresIMU(
          timeUSec = message.getLong("time_usec"),
          xAcc = message.getFloat("xacc"),
          yAcc = message.getFloat("yacc"),
          zAcc = message.getFloat("zacc"),
          xGyro = message.getFloat("xgyro"),
          yGyro = message.getFloat("ygyro"),
          zGyro = message.getFloat("zgyro"),
          xMag = message.getFloat("xmag"),
          yMag = message.getFloat("ymag"),
          zMag = message.getFloat("zmag"),
          absPressure = message.getFloat("abs_pressure"),
          diffPressure = message.getFloat("diff_pressure"),
          pressureAlt = message.getFloat("pressure_alt"),
          temperature = message.getFloat("temperature"),
          fieldsUpdated = message.getInt("fields_updated")
        )
      case 106 =>
        messages.OpticalFlowRad(
          timeUSec = message.getLong("time_usec"),
          sensorId = message.getInt("sensor_id"),
          integrationTimeUS = message.getInt("integration_time_us"),
          integratedX = message.getFloat("integrated_x"),
          integratedY = message.getFloat("integrated_y"),
          integratedXGyro = message.getFloat("integrated_xgyro"),
          integratedYGyro = message.getFloat("integrated_ygyro"),
          integratedZGyro = message.getFloat("integrated_zgyro"),
          temperature = message.getInt("temperature"),
          quality = message.getInt("quality"),
          timeDeltaDistanceUS = message.getInt("time_delta_distance_us"),
          distance = message.getFloat("distance")
        )
      case 107 =>
        messages.HILSensor(
          timeUSec = message.getLong("time_usec"),
          xAcc = message.getFloat("xacc"),
          yAcc = message.getFloat("yacc"),
          zAcc = message.getFloat("zacc"),
          xGyro = message.getFloat("xgyro"),
          yGyro = message.getFloat("ygyro"),
          zGyro = message.getFloat("zgyro"),
          xMag = message.getFloat("xmag"),
          yMag = message.getFloat("ymag"),
          zMag = message.getFloat("zmag"),
          absPressure = message.getFloat("abs_pressure"),
          diffPressure = message.getFloat("diff_pressure"),
          pressureAlt = message.getFloat("pressure_alt"),
          temperature = message.getFloat("temperature"),
          fieldsUpdated = message.getInt("fields_updated")
        )
      case 108 =>
        messages.SimState(
          q1 = message.getFloat("q1"),
          q2 = message.getFloat("q2"),
          q3 = message.getFloat("q3"),
          q4 = message.getFloat("q4"),
          roll = message.getFloat("roll"),
          pitch = message.getFloat("pitch"),
          yaw = message.getFloat("yaw"),
          xAcc = message.getFloat("xacc"),
          yAcc = message.getFloat("yacc"),
          zAcc = message.getFloat("zacc"),
          xGyro = message.getFloat("xgyro"),
          yGyro = message.getFloat("ygyro"),
          zGyro = message.getFloat("zgyro"),
          lat = message.getFloat("lat"),
          lon = message.getFloat("lon"),
          alt = message.getFloat("alt"),
          stdDevHorz = message.getFloat("std_dev_horz"),
          stdDevVert = message.getFloat("std_dev_vert"),
          vN = message.getFloat("vn"),
          vE = message.getFloat("ve"),
          vD = message.getFloat("vd")
        )
      case 109 =>
        messages.RadioStatus(
          rSSI = message.getInt("rssi"),
          remRSSI = message.getInt("remrssi"),
          tXBuf = message.getInt("txbuf"),
          noise = message.getInt("noise"),
          remNoise = message.getInt("remnoise"),
          rXErrors = message.getInt("rxerrors"),
          fixed = message.getInt("fixed")
        )
      case 110 =>
        messages.FileTransferProtocol(
          targetNetwork = message.getInt("target_network"),
          targetSystem = message.getInt("target_system"),
          targetComponent = message.getInt("target_component"),
          payload = message.get("payload").asInstanceOf[Array[AnyRef]].map(_.asInstanceOf[java.lang.Integer].toInt).toIndexedSeq
        )
      case 111 =>
        messages.Timesync(
          tc1 = message.getLong("tc1"),
          ts1 = message.getLong("ts1")
        )
      case 112 =>
        messages.CameraTrigger(
          timeUSec = message.getLong("time_usec"),
          seq = message.getInt("seq")
        )
      case 113 =>
        messages.HILGPS(
          timeUSec = message.getLong("time_usec"),
          fixType = message.getInt("fix_type"),
          lat = message.getInt("lat"),
          lon = message.getInt("lon"),
          alt = message.getInt("alt"),
          epH = message.getInt("eph"),
          epV = message.getInt("epv"),
          vel = message.getInt("vel"),
          vN = message.getInt("vn"),
          vE = message.getInt("ve"),
          vD = message.getInt("vd"),
          cOG = message.getInt("cog"),
          satellitesVisible = message.getInt("satellites_visible")
        )
      case 114 =>
        messages.HILOpticalFlow(
          timeUSec = message.getLong("time_usec"),
          sensorId = message.getInt("sensor_id"),
          integrationTimeUS = message.getInt("integration_time_us"),
          integratedX = message.getFloat("integrated_x"),
          integratedY = message.getFloat("integrated_y"),
          integratedXGyro = message.getFloat("integrated_xgyro"),
          integratedYGyro = message.getFloat("integrated_ygyro"),
          integratedZGyro = message.getFloat("integrated_zgyro"),
          temperature = message.getInt("temperature"),
          quality = message.getInt("quality"),
          timeDeltaDistanceUS = message.getInt("time_delta_distance_us"),
          distance = message.getFloat("distance")
        )
      case 115 =>
        messages.HILStateQuaternion(
          timeUSec = message.getLong("time_usec"),
          attitudeQuaternion = message.get("attitude_quaternion").asInstanceOf[Array[AnyRef]].map(_.asInstanceOf[java.lang.Float].toFloat).toIndexedSeq,
          rollSpeed = message.getFloat("rollspeed"),
          pitchSpeed = message.getFloat("pitchspeed"),
          yawSpeed = message.getFloat("yawspeed"),
          lat = message.getInt("lat"),
          lon = message.getInt("lon"),
          alt = message.getInt("alt"),
          vX = message.getInt("vx"),
          vY = message.getInt("vy"),
          vZ = message.getInt("vz"),
          indAirspeed = message.getInt("ind_airspeed"),
          trueAirspeed = message.getInt("true_airspeed"),
          xAcc = message.getInt("xacc"),
          yAcc = message.getInt("yacc"),
          zAcc = message.getInt("zacc")
        )
      case 116 =>
        messages.ScaledIMU2(
          timeBootMS = message.getInt("time_boot_ms"),
          xAcc = message.getInt("xacc"),
          yAcc = message.getInt("yacc"),
          zAcc = message.getInt("zacc"),
          xGyro = message.getInt("xgyro"),
          yGyro = message.getInt("ygyro"),
          zGyro = message.getInt("zgyro"),
          xMag = message.getInt("xmag"),
          yMag = message.getInt("ymag"),
          zMag = message.getInt("zmag")
        )
      case 117 =>
        messages.LogRequestList(
          targetSystem = message.getInt("target_system"),
          targetComponent = message.getInt("target_component"),
          start = message.getInt("start"),
          end = message.getInt("end")
        )
      case 118 =>
        messages.LogEntry(
          id = message.getInt("id"),
          numLogs = message.getInt("num_logs"),
          lastLogNum = message.getInt("last_log_num"),
          timeUTC = message.getInt("time_utc"),
          size = message.getInt("size")
        )
      case 119 =>
        messages.LogRequestData(
          targetSystem = message.getInt("target_system"),
          targetComponent = message.getInt("target_component"),
          id = message.getInt("id"),
          ofs = message.getInt("ofs"),
          count = message.getInt("count")
        )
      case 120 =>
        messages.LogData(
          id = message.getInt("id"),
          ofs = message.getInt("ofs"),
          count = message.getInt("count"),
          data = message.get("data").asInstanceOf[Array[AnyRef]].map(_.asInstanceOf[java.lang.Integer].toInt).toIndexedSeq
        )
      case 121 =>
        messages.LogErase(
          targetSystem = message.getInt("target_system"),
          targetComponent = message.getInt("target_component")
        )
      case 122 =>
        messages.LogRequestEnd(
          targetSystem = message.getInt("target_system"),
          targetComponent = message.getInt("target_component")
        )
      case 123 =>
        messages.GPSInjectData(
          targetSystem = message.getInt("target_system"),
          targetComponent = message.getInt("target_component"),
          len = message.getInt("len"),
          data = message.get("data").asInstanceOf[Array[AnyRef]].map(_.asInstanceOf[java.lang.Integer].toInt).toIndexedSeq
        )
      case 124 =>
        messages.GPS2Raw(
          timeUSec = message.getLong("time_usec"),
          fixType = GPSFixType(message.getInt("fix_type")),
          lat = message.getInt("lat"),
          lon = message.getInt("lon"),
          alt = message.getInt("alt"),
          epH = message.getInt("eph"),
          epV = message.getInt("epv"),
          vel = message.getInt("vel"),
          cOG = message.getInt("cog"),
          satellitesVisible = message.getInt("satellites_visible"),
          dGPSNumCh = message.getInt("dgps_numch"),
          dGPSAge = message.getInt("dgps_age")
        )
      case 125 =>
        messages.PowerStatus(
          vcc = message.getInt("Vcc"),
          vServo = message.getInt("Vservo"),
          flags = MAVPowerStatus.bitmask(message.getInt("flags"))
        )
      case 126 =>
        messages.SerialControl(
          device = SerialControlDev(message.getInt("device")),
          flags = SerialControlFlag.bitmask(message.getInt("flags")),
          timeout = message.getInt("timeout"),
          baudRate = message.getInt("baudrate"),
          count = message.getInt("count"),
          data = message.get("data").asInstanceOf[Array[AnyRef]].map(_.asInstanceOf[java.lang.Integer].toInt).toIndexedSeq
        )
      case 127 =>
        messages.GPSRTK(
          timeLastBaselineMS = message.getInt("time_last_baseline_ms"),
          rTKReceiverId = message.getInt("rtk_receiver_id"),
          wN = message.getInt("wn"),
          tOW = message.getInt("tow"),
          rTKHealth = message.getInt("rtk_health"),
          rTKRate = message.getInt("rtk_rate"),
          nSats = message.getInt("nsats"),
          baselineCoordsType = message.getInt("baseline_coords_type"),
          baselineAMM = message.getInt("baseline_a_mm"),
          baselineBMM = message.getInt("baseline_b_mm"),
          baselineCMM = message.getInt("baseline_c_mm"),
          accuracy = message.getInt("accuracy"),
          iARNumHypotheses = message.getInt("iar_num_hypotheses")
        )
      case 128 =>
        messages.GPS2RTK(
          timeLastBaselineMS = message.getInt("time_last_baseline_ms"),
          rTKReceiverId = message.getInt("rtk_receiver_id"),
          wN = message.getInt("wn"),
          tOW = message.getInt("tow"),
          rTKHealth = message.getInt("rtk_health"),
          rTKRate = message.getInt("rtk_rate"),
          nSats = message.getInt("nsats"),
          baselineCoordsType = message.getInt("baseline_coords_type"),
          baselineAMM = message.getInt("baseline_a_mm"),
          baselineBMM = message.getInt("baseline_b_mm"),
          baselineCMM = message.getInt("baseline_c_mm"),
          accuracy = message.getInt("accuracy"),
          iARNumHypotheses = message.getInt("iar_num_hypotheses")
        )
      case 129 =>
        messages.ScaledIMU3(
          timeBootMS = message.getInt("time_boot_ms"),
          xAcc = message.getInt("xacc"),
          yAcc = message.getInt("yacc"),
          zAcc = message.getInt("zacc"),
          xGyro = message.getInt("xgyro"),
          yGyro = message.getInt("ygyro"),
          zGyro = message.getInt("zgyro"),
          xMag = message.getInt("xmag"),
          yMag = message.getInt("ymag"),
          zMag = message.getInt("zmag")
        )
      case 130 =>
        messages.DataTransmissionHandshake(
          `type` = message.getInt("type"),
          size = message.getInt("size"),
          width = message.getInt("width"),
          height = message.getInt("height"),
          packets = message.getInt("packets"),
          payload = message.getInt("payload"),
          jPGQuality = message.getInt("jpg_quality")
        )
      case 131 =>
        messages.EncapsulatedData(
          seqNr = message.getInt("seqnr"),
          data = message.get("data").asInstanceOf[Array[AnyRef]].map(_.asInstanceOf[java.lang.Integer].toInt).toIndexedSeq
        )
      case 132 =>
        messages.DistanceSensor(
          timeBootMS = message.getInt("time_boot_ms"),
          minDistance = message.getInt("min_distance"),
          maxDistance = message.getInt("max_distance"),
          currentDistance = message.getInt("current_distance"),
          `type` = MAVDistanceSensor(message.getInt("type")),
          id = message.getInt("id"),
          orientation = MAVSensorOrientation(message.getInt("orientation")),
          covariance = message.getInt("covariance")
        )
      case 133 =>
        messages.TerrainRequest(
          lat = message.getInt("lat"),
          lon = message.getInt("lon"),
          gridSpacing = message.getInt("grid_spacing"),
          mask = message.getLong("mask")
        )
      case 134 =>
        messages.TerrainData(
          lat = message.getInt("lat"),
          lon = message.getInt("lon"),
          gridSpacing = message.getInt("grid_spacing"),
          gridBit = message.getInt("gridbit"),
          data = message.get("data").asInstanceOf[Array[AnyRef]].map(_.asInstanceOf[java.lang.Integer].toInt).toIndexedSeq
        )
      case 135 =>
        messages.TerrainCheck(
          lat = message.getInt("lat"),
          lon = message.getInt("lon")
        )
      case 136 =>
        messages.TerrainReport(
          lat = message.getInt("lat"),
          lon = message.getInt("lon"),
          spacing = message.getInt("spacing"),
          terrainHeight = message.getFloat("terrain_height"),
          currentHeight = message.getFloat("current_height"),
          pending = message.getInt("pending"),
          loaded = message.getInt("loaded")
        )
      case 137 =>
        messages.ScaledPressure2(
          timeBootMS = message.getInt("time_boot_ms"),
          pressAbs = message.getFloat("press_abs"),
          pressDiff = message.getFloat("press_diff"),
          temperature = message.getInt("temperature")
        )
      case 138 =>
        messages.AttPosMocap(
          timeUSec = message.getLong("time_usec"),
          q = message.get("q").asInstanceOf[Array[AnyRef]].map(_.asInstanceOf[java.lang.Float].toFloat).toIndexedSeq,
          x = message.getFloat("x"),
          y = message.getFloat("y"),
          z = message.getFloat("z")
        )
      case 139 =>
        messages.SetActuatorControlTarget(
          timeUSec = message.getLong("time_usec"),
          groupMLX = message.getInt("group_mlx"),
          targetSystem = message.getInt("target_system"),
          targetComponent = message.getInt("target_component"),
          controls = message.get("controls").asInstanceOf[Array[AnyRef]].map(_.asInstanceOf[java.lang.Float].toFloat).toIndexedSeq
        )
      case 140 =>
        messages.ActuatorControlTarget(
          timeUSec = message.getLong("time_usec"),
          groupMLX = message.getInt("group_mlx"),
          controls = message.get("controls").asInstanceOf[Array[AnyRef]].map(_.asInstanceOf[java.lang.Float].toFloat).toIndexedSeq
        )
      case 141 =>
        messages.Altitude(
          timeUSec = message.getLong("time_usec"),
          altitudeMonotonic = message.getFloat("altitude_monotonic"),
          altitudeAMSL = message.getFloat("altitude_amsl"),
          altitudeLocal = message.getFloat("altitude_local"),
          altitudeRelative = message.getFloat("altitude_relative"),
          altitudeTerrain = message.getFloat("altitude_terrain"),
          bottomClearance = message.getFloat("bottom_clearance")
        )
      case 142 =>
        messages.ResourceRequest(
          requestId = message.getInt("request_id"),
          uRIType = message.getInt("uri_type"),
          uRI = message.get("uri").asInstanceOf[Array[AnyRef]].map(_.asInstanceOf[java.lang.Integer].toInt).toIndexedSeq,
          transferType = message.getInt("transfer_type"),
          storage = message.get("storage").asInstanceOf[Array[AnyRef]].map(_.asInstanceOf[java.lang.Integer].toInt).toIndexedSeq
        )
      case 143 =>
        messages.ScaledPressure3(
          timeBootMS = message.getInt("time_boot_ms"),
          pressAbs = message.getFloat("press_abs"),
          pressDiff = message.getFloat("press_diff"),
          temperature = message.getInt("temperature")
        )
      case 144 =>
        messages.FollowTarget(
          timestamp = message.getLong("timestamp"),
          estCapabilities = message.getInt("est_capabilities"),
          lat = message.getInt("lat"),
          lon = message.getInt("lon"),
          alt = message.getFloat("alt"),
          vel = message.get("vel").asInstanceOf[Array[AnyRef]].map(_.asInstanceOf[java.lang.Float].toFloat).toIndexedSeq,
          acc = message.get("acc").asInstanceOf[Array[AnyRef]].map(_.asInstanceOf[java.lang.Float].toFloat).toIndexedSeq,
          attitudeQ = message.get("attitude_q").asInstanceOf[Array[AnyRef]].map(_.asInstanceOf[java.lang.Float].toFloat).toIndexedSeq,
          rates = message.get("rates").asInstanceOf[Array[AnyRef]].map(_.asInstanceOf[java.lang.Float].toFloat).toIndexedSeq,
          positionCov = message.get("position_cov").asInstanceOf[Array[AnyRef]].map(_.asInstanceOf[java.lang.Float].toFloat).toIndexedSeq,
          customState = message.getLong("custom_state")
        )
      case 146 =>
        messages.ControlSystemState(
          timeUSec = message.getLong("time_usec"),
          xAcc = message.getFloat("x_acc"),
          yAcc = message.getFloat("y_acc"),
          zAcc = message.getFloat("z_acc"),
          xVel = message.getFloat("x_vel"),
          yVel = message.getFloat("y_vel"),
          zVel = message.getFloat("z_vel"),
          xPos = message.getFloat("x_pos"),
          yPos = message.getFloat("y_pos"),
          zPos = message.getFloat("z_pos"),
          airspeed = message.getFloat("airspeed"),
          velVariance = message.get("vel_variance").asInstanceOf[Array[AnyRef]].map(_.asInstanceOf[java.lang.Float].toFloat).toIndexedSeq,
          posVariance = message.get("pos_variance").asInstanceOf[Array[AnyRef]].map(_.asInstanceOf[java.lang.Float].toFloat).toIndexedSeq,
          q = message.get("q").asInstanceOf[Array[AnyRef]].map(_.asInstanceOf[java.lang.Float].toFloat).toIndexedSeq,
          rollRate = message.getFloat("roll_rate"),
          pitchRate = message.getFloat("pitch_rate"),
          yawRate = message.getFloat("yaw_rate")
        )
      case 147 =>
        messages.BatteryStatus(
          id = message.getInt("id"),
          batteryFunction = MAVBatteryFunction(message.getInt("battery_function")),
          `type` = MAVBatteryType(message.getInt("type")),
          temperature = message.getInt("temperature"),
          voltages = message.get("voltages").asInstanceOf[Array[AnyRef]].map(_.asInstanceOf[java.lang.Integer].toInt).toIndexedSeq,
          currentBattery = message.getInt("current_battery"),
          currentConsumed = message.getInt("current_consumed"),
          energyConsumed = message.getInt("energy_consumed"),
          batteryRemaining = message.getInt("battery_remaining")
        )
      case 148 =>
        messages.AutopilotVersion(
          capabilities = MAVProtocolCapability.bitmask(message.getInt("capabilities")),
          flightSWVersion = message.getInt("flight_sw_version"),
          middlewareSWVersion = message.getInt("middleware_sw_version"),
          oSSWVersion = message.getInt("os_sw_version"),
          boardVersion = message.getInt("board_version"),
          flightCustomVersion = message.get("flight_custom_version").asInstanceOf[Array[AnyRef]].map(_.asInstanceOf[java.lang.Integer].toInt).toIndexedSeq,
          middlewareCustomVersion = message.get("middleware_custom_version").asInstanceOf[Array[AnyRef]].map(_.asInstanceOf[java.lang.Integer].toInt).toIndexedSeq,
          oSCustomVersion = message.get("os_custom_version").asInstanceOf[Array[AnyRef]].map(_.asInstanceOf[java.lang.Integer].toInt).toIndexedSeq,
          vendorId = message.getInt("vendor_id"),
          productId = message.getInt("product_id"),
          uid = message.getLong("uid"),
          uid2 = message.get("uid2").asInstanceOf[Array[AnyRef]].map(_.asInstanceOf[java.lang.Integer].toInt).toIndexedSeq
        )
      case 149 =>
        messages.LandingTarget(
          timeUSec = message.getLong("time_usec"),
          targetNum = message.getInt("target_num"),
          frame = MAVFrame(message.getInt("frame")),
          angleX = message.getFloat("angle_x"),
          angleY = message.getFloat("angle_y"),
          distance = message.getFloat("distance"),
          sizeX = message.getFloat("size_x"),
          sizeY = message.getFloat("size_y"),
          x = message.getFloat("x"),
          y = message.getFloat("y"),
          z = message.getFloat("z"),
          q = message.get("q").asInstanceOf[Array[AnyRef]].map(_.asInstanceOf[java.lang.Float].toFloat).toIndexedSeq,
          `type` = LandingTargetType(message.getInt("type")),
          positionValid = message.getInt("position_valid")
        )
      case 230 =>
        messages.EstimatorStatus(
          timeUSec = message.getLong("time_usec"),
          flags = EstimatorStatusFlags.bitmask(message.getInt("flags")),
          velRatio = message.getFloat("vel_ratio"),
          posHorizRatio = message.getFloat("pos_horiz_ratio"),
          posVertRatio = message.getFloat("pos_vert_ratio"),
          magRatio = message.getFloat("mag_ratio"),
          hAGLRatio = message.getFloat("hagl_ratio"),
          tASRatio = message.getFloat("tas_ratio"),
          posHorizAccuracy = message.getFloat("pos_horiz_accuracy"),
          posVertAccuracy = message.getFloat("pos_vert_accuracy")
        )
      case 231 =>
        messages.WindCov(
          timeUSec = message.getLong("time_usec"),
          windX = message.getFloat("wind_x"),
          windY = message.getFloat("wind_y"),
          windZ = message.getFloat("wind_z"),
          varHoriz = message.getFloat("var_horiz"),
          varVert = message.getFloat("var_vert"),
          windAlt = message.getFloat("wind_alt"),
          horizAccuracy = message.getFloat("horiz_accuracy"),
          vertAccuracy = message.getFloat("vert_accuracy")
        )
      case 232 =>
        messages.GPSInput(
          timeUSec = message.getLong("time_usec"),
          gPSId = message.getInt("gps_id"),
          ignoreFlags = GPSInputIgnoreFlags.bitmask(message.getInt("ignore_flags")),
          timeWeekMS = message.getInt("time_week_ms"),
          timeWeek = message.getInt("time_week"),
          fixType = message.getInt("fix_type"),
          lat = message.getInt("lat"),
          lon = message.getInt("lon"),
          alt = message.getFloat("alt"),
          hDOP = message.getFloat("hdop"),
          vDOP = message.getFloat("vdop"),
          vN = message.getFloat("vn"),
          vE = message.getFloat("ve"),
          vD = message.getFloat("vd"),
          speedAccuracy = message.getFloat("speed_accuracy"),
          horizAccuracy = message.getFloat("horiz_accuracy"),
          vertAccuracy = message.getFloat("vert_accuracy"),
          satellitesVisible = message.getInt("satellites_visible")
        )
      case 233 =>
        messages.GPSRTCMData(
          flags = message.getInt("flags"),
          len = message.getInt("len"),
          data = message.get("data").asInstanceOf[Array[AnyRef]].map(_.asInstanceOf[java.lang.Integer].toInt).toIndexedSeq
        )
      case 234 =>
        messages.HighLatency(
          baseMode = MAVModeFlag.bitmask(message.getInt("base_mode")),
          customMode = message.getInt("custom_mode"),
          landedState = MAVLandedState(message.getInt("landed_state")),
          roll = message.getInt("roll"),
          pitch = message.getInt("pitch"),
          heading = message.getInt("heading"),
          throttle = message.getInt("throttle"),
          headingSp = message.getInt("heading_sp"),
          latitude = message.getInt("latitude"),
          longitude = message.getInt("longitude"),
          altitudeAMSL = message.getInt("altitude_amsl"),
          altitudeSp = message.getInt("altitude_sp"),
          airspeed = message.getInt("airspeed"),
          airspeedSp = message.getInt("airspeed_sp"),
          groundSpeed = message.getInt("groundspeed"),
          climbRate = message.getInt("climb_rate"),
          gPSNsat = message.getInt("gps_nsat"),
          gPSFixType = GPSFixType(message.getInt("gps_fix_type")),
          batteryRemaining = message.getInt("battery_remaining"),
          temperature = message.getInt("temperature"),
          temperatureAir = message.getInt("temperature_air"),
          failsafe = message.getInt("failsafe"),
          wpNum = message.getInt("wp_num"),
          wpDistance = message.getInt("wp_distance")
        )
      case 241 =>
        messages.Vibration(
          timeUSec = message.getLong("time_usec"),
          vibrationX = message.getFloat("vibration_x"),
          vibrationY = message.getFloat("vibration_y"),
          vibrationZ = message.getFloat("vibration_z"),
          clipping0 = message.getInt("clipping_0"),
          clipping1 = message.getInt("clipping_1"),
          clipping2 = message.getInt("clipping_2")
        )
      case 242 =>
        messages.HomePosition(
          latitude = message.getInt("latitude"),
          longitude = message.getInt("longitude"),
          altitude = message.getInt("altitude"),
          x = message.getFloat("x"),
          y = message.getFloat("y"),
          z = message.getFloat("z"),
          q = message.get("q").asInstanceOf[Array[AnyRef]].map(_.asInstanceOf[java.lang.Float].toFloat).toIndexedSeq,
          approachX = message.getFloat("approach_x"),
          approachY = message.getFloat("approach_y"),
          approachZ = message.getFloat("approach_z"),
          timeUSec = message.getLong("time_usec")
        )
      case 243 =>
        messages.SetHomePosition(
          targetSystem = message.getInt("target_system"),
          latitude = message.getInt("latitude"),
          longitude = message.getInt("longitude"),
          altitude = message.getInt("altitude"),
          x = message.getFloat("x"),
          y = message.getFloat("y"),
          z = message.getFloat("z"),
          q = message.get("q").asInstanceOf[Array[AnyRef]].map(_.asInstanceOf[java.lang.Float].toFloat).toIndexedSeq,
          approachX = message.getFloat("approach_x"),
          approachY = message.getFloat("approach_y"),
          approachZ = message.getFloat("approach_z"),
          timeUSec = message.getLong("time_usec")
        )
      case 244 =>
        messages.MessageInterval(
          messageId = message.getInt("message_id"),
          intervalUS = message.getInt("interval_us")
        )
      case 245 =>
        messages.ExtendedSysState(
          vTOLState = MAVVTOLState(message.getInt("vtol_state")),
          landedState = MAVLandedState(message.getInt("landed_state"))
        )
      case 246 =>
        messages.ADSBVehicle(
          icaoAddress = message.getInt("ICAO_address"),
          lat = message.getInt("lat"),
          lon = message.getInt("lon"),
          altitudeType = ADSBAltitudeType(message.getInt("altitude_type")),
          altitude = message.getInt("altitude"),
          heading = message.getInt("heading"),
          horVelocity = message.getInt("hor_velocity"),
          verVelocity = message.getInt("ver_velocity"),
          callsign = message.getString("callsign"),
          emitterType = ADSBEmitterType(message.getInt("emitter_type")),
          tslc = message.getInt("tslc"),
          flags = ADSBFlags.bitmask(message.getInt("flags")),
          squawk = message.getInt("squawk")
        )
      case 247 =>
        messages.Collision(
          src = MAVCollisionSrc(message.getInt("src")),
          id = message.getInt("id"),
          action = MAVCollisionAction(message.getInt("action")),
          threatLevel = MAVCollisionThreatLevel(message.getInt("threat_level")),
          timeToMinimumDelta = message.getFloat("time_to_minimum_delta"),
          altitudeMinimumDelta = message.getFloat("altitude_minimum_delta"),
          horizontalMinimumDelta = message.getFloat("horizontal_minimum_delta")
        )
      case 248 =>
        messages.V2Extension(
          targetNetwork = message.getInt("target_network"),
          targetSystem = message.getInt("target_system"),
          targetComponent = message.getInt("target_component"),
          messageType = message.getInt("message_type"),
          payload = message.get("payload").asInstanceOf[Array[AnyRef]].map(_.asInstanceOf[java.lang.Integer].toInt).toIndexedSeq
        )
      case 249 =>
        messages.MemoryVect(
          address = message.getInt("address"),
          ver = message.getInt("ver"),
          `type` = message.getInt("type"),
          value = message.get("value").asInstanceOf[Array[AnyRef]].map(_.asInstanceOf[java.lang.Integer].toInt).toIndexedSeq
        )
      case 250 =>
        messages.DebugVect(
          name = message.getString("name"),
          timeUSec = message.getLong("time_usec"),
          x = message.getFloat("x"),
          y = message.getFloat("y"),
          z = message.getFloat("z")
        )
      case 251 =>
        messages.NamedValueFloat(
          timeBootMS = message.getInt("time_boot_ms"),
          name = message.getString("name"),
          value = message.getFloat("value")
        )
      case 252 =>
        messages.NamedValueInt(
          timeBootMS = message.getInt("time_boot_ms"),
          name = message.getString("name"),
          value = message.getInt("value")
        )
      case 253 =>
        messages.Statustext(
          severity = MAVSeverity(message.getInt("severity")),
          text = message.getString("text")
        )
      case 254 =>
        messages.Debug(
          timeBootMS = message.getInt("time_boot_ms"),
          ind = message.getInt("ind"),
          value = message.getFloat("value")
        )
      case 256 =>
        messages.SetupSigning(
          targetSystem = message.getInt("target_system"),
          targetComponent = message.getInt("target_component"),
          secretKey = message.get("secret_key").asInstanceOf[Array[AnyRef]].map(_.asInstanceOf[java.lang.Integer].toInt).toIndexedSeq,
          initialTimestamp = message.getLong("initial_timestamp")
        )
      case 257 =>
        messages.ButtonChange(
          timeBootMS = message.getInt("time_boot_ms"),
          lastChangeMS = message.getInt("last_change_ms"),
          state = message.getInt("state")
        )
      case 258 =>
        messages.PlayTune(
          targetSystem = message.getInt("target_system"),
          targetComponent = message.getInt("target_component"),
          tune = message.getString("tune")
        )
      case 259 =>
        messages.CameraInformation(
          timeBootMS = message.getInt("time_boot_ms"),
          vendorName = message.get("vendor_name").asInstanceOf[Array[AnyRef]].map(_.asInstanceOf[java.lang.Integer].toInt).toIndexedSeq,
          modelName = message.get("model_name").asInstanceOf[Array[AnyRef]].map(_.asInstanceOf[java.lang.Integer].toInt).toIndexedSeq,
          firmwareVersion = message.getInt("firmware_version"),
          focalLength = message.getFloat("focal_length"),
          sensorSizeH = message.getFloat("sensor_size_h"),
          sensorSizeV = message.getFloat("sensor_size_v"),
          resolutionH = message.getInt("resolution_h"),
          resolutionV = message.getInt("resolution_v"),
          lensId = message.getInt("lens_id"),
          flags = CameraCapFlags(message.getInt("flags")),
          camDefinitionVersion = message.getInt("cam_definition_version"),
          camDefinitionURI = message.getString("cam_definition_uri")
        )
      case 260 =>
        messages.CameraSettings(
          timeBootMS = message.getInt("time_boot_ms"),
          modeId = CameraMode(message.getInt("mode_id"))
        )
      case 261 =>
        messages.StorageInformation(
          timeBootMS = message.getInt("time_boot_ms"),
          storageId = message.getInt("storage_id"),
          storageCount = message.getInt("storage_count"),
          status = message.getInt("status"),
          totalCapacity = message.getFloat("total_capacity"),
          usedCapacity = message.getFloat("used_capacity"),
          availableCapacity = message.getFloat("available_capacity"),
          readSpeed = message.getFloat("read_speed"),
          writeSpeed = message.getFloat("write_speed")
        )
      case 262 =>
        messages.CameraCaptureStatus(
          timeBootMS = message.getInt("time_boot_ms"),
          imageStatus = message.getInt("image_status"),
          videoStatus = message.getInt("video_status"),
          imageInterval = message.getFloat("image_interval"),
          recordingTimeMS = message.getInt("recording_time_ms"),
          availableCapacity = message.getFloat("available_capacity")
        )
      case 263 =>
        messages.CameraImageCaptured(
          timeBootMS = message.getInt("time_boot_ms"),
          timeUTC = message.getLong("time_utc"),
          cameraId = message.getInt("camera_id"),
          lat = message.getInt("lat"),
          lon = message.getInt("lon"),
          alt = message.getInt("alt"),
          relativeAlt = message.getInt("relative_alt"),
          q = message.get("q").asInstanceOf[Array[AnyRef]].map(_.asInstanceOf[java.lang.Float].toFloat).toIndexedSeq,
          imageIndex = message.getInt("image_index"),
          captureResult = message.getInt("capture_result"),
          fileURL = message.getString("file_url")
        )
      case 264 =>
        messages.FlightInformation(
          timeBootMS = message.getInt("time_boot_ms"),
          armingTimeUTC = message.getLong("arming_time_utc"),
          takeoffTimeUTC = message.getLong("takeoff_time_utc"),
          flightUUID = message.getLong("flight_uuid")
        )
      case 265 =>
        messages.MountOrientation(
          timeBootMS = message.getInt("time_boot_ms"),
          roll = message.getFloat("roll"),
          pitch = message.getFloat("pitch"),
          yaw = message.getFloat("yaw")
        )
      case 266 =>
        messages.LoggingData(
          targetSystem = message.getInt("target_system"),
          targetComponent = message.getInt("target_component"),
          sequence = message.getInt("sequence"),
          length = message.getInt("length"),
          firstMessageOffset = message.getInt("first_message_offset"),
          data = message.get("data").asInstanceOf[Array[AnyRef]].map(_.asInstanceOf[java.lang.Integer].toInt).toIndexedSeq
        )
      case 267 =>
        messages.LoggingDataAcked(
          targetSystem = message.getInt("target_system"),
          targetComponent = message.getInt("target_component"),
          sequence = message.getInt("sequence"),
          length = message.getInt("length"),
          firstMessageOffset = message.getInt("first_message_offset"),
          data = message.get("data").asInstanceOf[Array[AnyRef]].map(_.asInstanceOf[java.lang.Integer].toInt).toIndexedSeq
        )
      case 268 =>
        messages.LoggingAck(
          targetSystem = message.getInt("target_system"),
          targetComponent = message.getInt("target_component"),
          sequence = message.getInt("sequence")
        )
      case 269 =>
        messages.VideoStreamInformation(
          cameraId = message.getInt("camera_id"),
          status = message.getInt("status"),
          frameRate = message.getFloat("framerate"),
          resolutionH = message.getInt("resolution_h"),
          resolutionV = message.getInt("resolution_v"),
          bitRate = message.getInt("bitrate"),
          rotation = message.getInt("rotation"),
          uRI = message.getString("uri")
        )
      case 270 =>
        messages.SetVideoStreamSettings(
          targetSystem = message.getInt("target_system"),
          targetComponent = message.getInt("target_component"),
          cameraId = message.getInt("camera_id"),
          frameRate = message.getFloat("framerate"),
          resolutionH = message.getInt("resolution_h"),
          resolutionV = message.getInt("resolution_v"),
          bitRate = message.getInt("bitrate"),
          rotation = message.getInt("rotation"),
          uRI = message.getString("uri")
        )
      case 299 =>
        messages.WifiConfigAp(
          ssid = message.getString("ssid"),
          password = message.getString("password")
        )
      case 300 =>
        messages.ProtocolVersion(
          version = message.getInt("version"),
          minVersion = message.getInt("min_version"),
          maxVersion = message.getInt("max_version"),
          specVersionHash = message.get("spec_version_hash").asInstanceOf[Array[AnyRef]].map(_.asInstanceOf[java.lang.Integer].toInt).toIndexedSeq,
          libraryVersionHash = message.get("library_version_hash").asInstanceOf[Array[AnyRef]].map(_.asInstanceOf[java.lang.Integer].toInt).toIndexedSeq
        )
      case 310 =>
        messages.UAVCANNodeStatus(
          timeUSec = message.getLong("time_usec"),
          uptimeSec = message.getInt("uptime_sec"),
          health = UAVCANNodeHealth(message.getInt("health")),
          mode = UAVCANNodeMode(message.getInt("mode")),
          subMode = message.getInt("sub_mode"),
          vendorSpecificStatusCode = message.getInt("vendor_specific_status_code")
        )
      case 311 =>
        messages.UAVCANNodeInfo(
          timeUSec = message.getLong("time_usec"),
          uptimeSec = message.getInt("uptime_sec"),
          name = message.getString("name"),
          hWVersionMajor = message.getInt("hw_version_major"),
          hWVersionMinor = message.getInt("hw_version_minor"),
          hWUniqueId = message.get("hw_unique_id").asInstanceOf[Array[AnyRef]].map(_.asInstanceOf[java.lang.Integer].toInt).toIndexedSeq,
          sWVersionMajor = message.getInt("sw_version_major"),
          sWVersionMinor = message.getInt("sw_version_minor"),
          sWVcsCommit = message.getInt("sw_vcs_commit")
        )
      case 320 =>
        messages.ParamExtRequestRead(
          targetSystem = message.getInt("target_system"),
          targetComponent = message.getInt("target_component"),
          paramId = message.getString("param_id"),
          paramIndex = message.getInt("param_index")
        )
      case 321 =>
        messages.ParamExtRequestList(
          targetSystem = message.getInt("target_system"),
          targetComponent = message.getInt("target_component")
        )
      case 322 =>
        messages.ParamExtValue(
          paramId = message.getString("param_id"),
          paramValue = message.getString("param_value"),
          paramType = MAVParamExtType(message.getInt("param_type")),
          paramCount = message.getInt("param_count"),
          paramIndex = message.getInt("param_index")
        )
      case 323 =>
        messages.ParamExtSet(
          targetSystem = message.getInt("target_system"),
          targetComponent = message.getInt("target_component"),
          paramId = message.getString("param_id"),
          paramValue = message.getString("param_value"),
          paramType = MAVParamExtType(message.getInt("param_type"))
        )
      case 324 =>
        messages.ParamExtAck(
          paramId = message.getString("param_id"),
          paramValue = message.getString("param_value"),
          paramType = MAVParamExtType(message.getInt("param_type")),
          paramResult = ParamAck(message.getInt("param_result"))
        )
      case 330 =>
        messages.ObstacleDistance(
          timeUSec = message.getLong("time_usec"),
          sensorType = MAVDistanceSensor(message.getInt("sensor_type")),
          distances = message.get("distances").asInstanceOf[Array[AnyRef]].map(_.asInstanceOf[java.lang.Integer].toInt).toIndexedSeq,
          increment = message.getInt("increment"),
          minDistance = message.getInt("min_distance"),
          maxDistance = message.getInt("max_distance")
        )
      case error => throw new MatchError(error)
    }
  }
}
