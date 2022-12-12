package com.peterpotts.trax.mavlink.model

import com.peterpotts.trax.mavlink.model.MAVEnums._

/**
  * @author Peter Potts
  */
case class MAVMessages(systemId: Int, componentId: MAVComponent, protocolVersion: Int) {

  /**
    * 0 HEARTBEAT
    *
    * The heartbeat message shows that a system is present and responding. The type of the MAV and Autopilot hardware allow the receiving system to treat further messages from this system appropriate (e.g. by laying out the user interface based on the autopilot).
    *
    * @param `type`       Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
    * @param autopilot    Autopilot type / class. defined in MAV_AUTOPILOT ENUM
    * @param baseMode     System mode bitfield, see MAV_MODE_FLAG ENUM in mavlink/include/mavlink_types.h
    * @param customMode   A bitfield for use for autopilot-specific flags.
    * @param systemStatus System status flag, see MAV_STATE ENUM
    */
  case class Heartbeat(
    `type`: MAVType,
    autopilot: MAVAutopilot,
    baseMode: Set[MAVModeFlag],
    customMode: Int,
    systemStatus: MAVState
  ) extends MAVMessage("HEARTBEAT", systemId, componentId, protocolVersion) {
    set("type", `type`.value)
    set("autopilot", autopilot.value)
    set("base_mode", baseMode.foldLeft(0)(_ | _.value))
    set("custom_mode", customMode)
    set("system_status", systemStatus.value)

    override def toString: String =
      List(
        s"systemId = $systemId",
        s"componentId = ${componentId.value}",
        s"protocolVersion = $protocolVersion",
        s"type = ${`type`}",
        s"autopilot = $autopilot",
        s"baseMode = $baseMode",
        s"customMode = $customMode",
        s"systemStatus = $systemStatus"
      ).mkString("Heartbeat(", ", ", ")")
  }

  /**
    * 1 SYS_STATUS
    *
    * The general system state. If the system is following the MAVLink standard, the system state is mainly defined by three orthogonal states/modes: The system mode, which is either LOCKED (motors shut down and locked), MANUAL (system under RC control), GUIDED (system with autonomous position control, position setpoint controlled manually) or AUTO (system guided by path/waypoint planner). The NAV_MODE defined the current flight state: LIFTOFF (often an open-loop maneuver), LANDING, WAYPOINTS or VECTOR. This represents the internal navigation state machine. The system status shows whether the system is currently active or not and if an emergency occured. During the CRITICAL and EMERGENCY states the MAV is still considered to be active, but should start emergency procedures autonomously. After a failure occured it should first move from active to critical to allow manual intervention and then move to emergency after a certain timeout.
    *
    * @param onboardControlSensorsPresent Bitmask showing which onboard controllers and sensors are present. Value of 0: not present. Value of 1: present. Indices defined by ENUM MAV_SYS_STATUS_SENSOR
    * @param onboardControlSensorsEnabled Bitmask showing which onboard controllers and sensors are enabled:  Value of 0: not enabled. Value of 1: enabled. Indices defined by ENUM MAV_SYS_STATUS_SENSOR
    * @param onboardControlSensorsHealth  Bitmask showing which onboard controllers and sensors are operational or have an error:  Value of 0: not enabled. Value of 1: enabled. Indices defined by ENUM MAV_SYS_STATUS_SENSOR
    * @param load                         Maximum usage in percent of the mainloop time, (0%: 0, 100%: 1000) should be always below 1000
    * @param voltageBattery               Battery voltage, in millivolts (1 = 1 millivolt)
    * @param currentBattery               Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current
    * @param batteryRemaining             Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot estimate the remaining battery
    * @param dropRateComm                 Communication drops in percent, (0%: 0, 100%: 10'000), (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
    * @param errorsComm                   Communication errors (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
    * @param errorsCount1                 Autopilot-specific errors
    * @param errorsCount2                 Autopilot-specific errors
    * @param errorsCount3                 Autopilot-specific errors
    * @param errorsCount4                 Autopilot-specific errors
    */
  case class SysStatus(
    onboardControlSensorsPresent: Set[MAVSysStatusSensor],
    onboardControlSensorsEnabled: Set[MAVSysStatusSensor],
    onboardControlSensorsHealth: Set[MAVSysStatusSensor],
    load: Int,
    voltageBattery: Int,
    currentBattery: Int,
    batteryRemaining: Int,
    dropRateComm: Int,
    errorsComm: Int,
    errorsCount1: Int,
    errorsCount2: Int,
    errorsCount3: Int,
    errorsCount4: Int
  ) extends MAVMessage("SYS_STATUS", systemId, componentId, protocolVersion) {
    set("onboard_control_sensors_present", onboardControlSensorsPresent.foldLeft(0)(_ | _.value))
    set("onboard_control_sensors_enabled", onboardControlSensorsEnabled.foldLeft(0)(_ | _.value))
    set("onboard_control_sensors_health", onboardControlSensorsHealth.foldLeft(0)(_ | _.value))
    set("load", load)
    set("voltage_battery", voltageBattery)
    set("current_battery", currentBattery)
    set("battery_remaining", batteryRemaining)
    set("drop_rate_comm", dropRateComm)
    set("errors_comm", errorsComm)
    set("errors_count1", errorsCount1)
    set("errors_count2", errorsCount2)
    set("errors_count3", errorsCount3)
    set("errors_count4", errorsCount4)

    override def toString: String =
      List(
        s"onboardControlSensorsPresent = $onboardControlSensorsPresent",
        s"onboardControlSensorsEnabled = $onboardControlSensorsEnabled",
        s"onboardControlSensorsHealth = $onboardControlSensorsHealth",
        s"load = $load",
        s"voltageBattery = $voltageBattery",
        s"currentBattery = $currentBattery",
        s"batteryRemaining = $batteryRemaining",
        s"dropRateComm = $dropRateComm",
        s"errorsComm = $errorsComm",
        s"errorsCount1 = $errorsCount1",
        s"errorsCount2 = $errorsCount2",
        s"errorsCount3 = $errorsCount3",
        s"errorsCount4 = $errorsCount4"
      ).mkString("SysStatus(", ", ", ")")
  }

  /**
    * 2 SYSTEM_TIME
    *
    * The system time is the time of the master clock, typically the computer clock of the main onboard computer.
    *
    * @param timeUnixUSec Timestamp of the master clock in microseconds since UNIX epoch.
    * @param timeBootMS   Timestamp of the component clock since boot time in milliseconds.
    */
  case class SystemTime(
    timeUnixUSec: Long,
    timeBootMS: Int
  ) extends MAVMessage("SYSTEM_TIME", systemId, componentId, protocolVersion) {
    set("time_unix_usec", timeUnixUSec)
    set("time_boot_ms", timeBootMS)

    override def toString: String =
      List(
        s"timeUnixUSec = $timeUnixUSec",
        s"timeBootMS = $timeBootMS"
      ).mkString("SystemTime(", ", ", ")")
  }

  /**
    * 4 PING
    *
    * A ping message either requesting or responding to a ping. This allows to measure the system latencies, including serial port, radio modem and UDP connections.
    *
    * @param timeUSec        Unix timestamp in microseconds or since system boot if smaller than MAVLink epoch (1.1.2009)
    * @param seq             PING sequence
    * @param targetSystem    0: request ping from all receiving systems, if greater than 0: message is a ping response and number is the system id of the requesting system
    * @param targetComponent 0: request ping from all receiving components, if greater than 0: message is a ping response and number is the system id of the requesting system
    */
  case class Ping(
    timeUSec: Long,
    seq: Int,
    targetSystem: Int,
    targetComponent: Int
  ) extends MAVMessage("PING", systemId, componentId, protocolVersion) {
    set("time_usec", timeUSec)
    set("seq", seq)
    set("target_system", targetSystem)
    set("target_component", targetComponent)

    override def toString: String =
      List(
        s"timeUSec = $timeUSec",
        s"seq = $seq",
        s"targetSystem = $targetSystem",
        s"targetComponent = $targetComponent"
      ).mkString("Ping(", ", ", ")")
  }

  /**
    * 5 CHANGE_OPERATOR_CONTROL
    *
    * Request to control this MAV
    *
    * @param targetSystem   System the GCS requests control for
    * @param controlRequest 0: request control of this MAV, 1: Release control of this MAV
    * @param version        0: key as plaintext, 1-255: future, different hashing/encryption variants. The GCS should in general use the safest mode possible initially and then gradually move down the encryption level if it gets a NACK message indicating an encryption mismatch.
    * @param passkey        Password / Key, depending on version plaintext or encrypted. 25 or less characters, NULL terminated. The characters may involve A-Z, a-z, 0-9, and "!?,.-"
    */
  case class ChangeOperatorControl(
    targetSystem: Int,
    controlRequest: Int,
    version: Int,
    passkey: String
  ) extends MAVMessage("CHANGE_OPERATOR_CONTROL", systemId, componentId, protocolVersion) {
    set("target_system", targetSystem)
    set("control_request", controlRequest)
    set("version", version)
    set("passkey", passkey)

    override def toString: String =
      List(
        s"targetSystem = $targetSystem",
        s"controlRequest = $controlRequest",
        s"version = $version",
        s"passkey = $passkey"
      ).mkString("ChangeOperatorControl(", ", ", ")")
  }

  /**
    * 6 CHANGE_OPERATOR_CONTROL_ACK
    *
    * Accept / deny control of this MAV
    *
    * @param gCSSystemId    ID of the GCS this message
    * @param controlRequest 0: request control of this MAV, 1: Release control of this MAV
    * @param ack            0: ACK, 1: NACK: Wrong passkey, 2: NACK: Unsupported passkey encryption method, 3: NACK: Already under control
    */
  case class ChangeOperatorControlAck(
    gCSSystemId: Int,
    controlRequest: Int,
    ack: Int
  ) extends MAVMessage("CHANGE_OPERATOR_CONTROL_ACK", systemId, componentId, protocolVersion) {
    set("gcs_system_id", gCSSystemId)
    set("control_request", controlRequest)
    set("ack", ack)

    override def toString: String =
      List(
        s"gCSSystemId = $gCSSystemId",
        s"controlRequest = $controlRequest",
        s"ack = $ack"
      ).mkString("ChangeOperatorControlAck(", ", ", ")")
  }

  /**
    * 7 AUTH_KEY
    *
    * Emit an encrypted signature / key identifying this system. PLEASE NOTE: This protocol has been kept simple, so transmitting the key requires an encrypted channel for true safety.
    *
    * @param key key
    */
  case class AuthKey(
    key: String
  ) extends MAVMessage("AUTH_KEY", systemId, componentId, protocolVersion) {
    set("key", key)

    override def toString: String =
      List(
        s"key = $key"
      ).mkString("AuthKey(", ", ", ")")
  }

  /**
    * 11 SET_MODE
    *
    * THIS INTERFACE IS DEPRECATED. USE COMMAND_LONG with MAV_CMD_DO_SET_MODE INSTEAD. Set the system mode, as defined by enum MAV_MODE. There is no target component id as the mode is by definition for the overall aircraft, not only for one component.
    *
    * @param targetSystem The system setting the mode
    * @param baseMode     The new base mode
    * @param customMode   The new autopilot-specific mode. This field can be ignored by an autopilot.
    */
  case class SetMode(
    targetSystem: Int,
    baseMode: MAVMode,
    customMode: Int
  ) extends MAVMessage("SET_MODE", systemId, componentId, protocolVersion) {
    set("target_system", targetSystem)
    set("base_mode", baseMode.value)
    set("custom_mode", customMode)

    override def toString: String =
      List(
        s"targetSystem = $targetSystem",
        s"baseMode = $baseMode",
        s"customMode = $customMode"
      ).mkString("SetMode(", ", ", ")")
  }

  /**
    * 20 PARAM_REQUEST_READ
    *
    * Request to read the onboard parameter with the param_id string id. Onboard parameters are stored as key[const char*] -> value[float]. This allows to send a parameter to any other component (such as the GCS) without the need of previous knowledge of possible parameter names. Thus the same GCS can store different parameters for different autopilots. See also http://qgroundcontrol.org/parameter_interface for a full documentation of QGroundControl and IMU code.
    *
    * @param targetSystem    System ID
    * @param targetComponent Component ID
    * @param paramId         Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
    * @param paramIndex      Parameter index. Send -1 to use the param ID field as identifier (else the param id will be ignored)
    */
  case class ParamRequestRead(
    targetSystem: Int,
    targetComponent: Int,
    paramId: String,
    paramIndex: Int
  ) extends MAVMessage("PARAM_REQUEST_READ", systemId, componentId, protocolVersion) {
    set("target_system", targetSystem)
    set("target_component", targetComponent)
    set("param_id", paramId)
    set("param_index", paramIndex)

    override def toString: String =
      List(
        s"targetSystem = $targetSystem",
        s"targetComponent = $targetComponent",
        s"paramId = $paramId",
        s"paramIndex = $paramIndex"
      ).mkString("ParamRequestRead(", ", ", ")")
  }

  /**
    * 21 PARAM_REQUEST_LIST
    *
    * Request all parameters of this component. After this request, all parameters are emitted.
    *
    * @param targetSystem    System ID
    * @param targetComponent Component ID
    */
  case class ParamRequestList(
    targetSystem: Int,
    targetComponent: Int
  ) extends MAVMessage("PARAM_REQUEST_LIST", systemId, componentId, protocolVersion) {
    set("target_system", targetSystem)
    set("target_component", targetComponent)

    override def toString: String =
      List(
        s"targetSystem = $targetSystem",
        s"targetComponent = $targetComponent"
      ).mkString("ParamRequestList(", ", ", ")")
  }

  /**
    * 22 PARAM_VALUE
    *
    * Emit the value of a onboard parameter. The inclusion of param_count and param_index in the message allows the recipient to keep track of received parameters and allows him to re-request missing parameters after a loss or timeout.
    *
    * @param paramId    Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
    * @param paramValue Onboard parameter value
    * @param paramType  Onboard parameter type: see the MAV_PARAM_TYPE enum for supported data types.
    * @param paramCount Total number of onboard parameters
    * @param paramIndex Index of this onboard parameter
    */
  case class ParamValue(
    paramId: String,
    paramValue: Float,
    paramType: MAVParamType,
    paramCount: Int,
    paramIndex: Int
  ) extends MAVMessage("PARAM_VALUE", systemId, componentId, protocolVersion) {
    set("param_id", paramId)
    set("param_value", paramValue)
    set("param_type", paramType.value)
    set("param_count", paramCount)
    set("param_index", paramIndex)

    override def toString: String =
      List(
        s"paramId = $paramId",
        s"paramValue = $paramValue",
        s"paramType = $paramType",
        s"paramCount = $paramCount",
        s"paramIndex = $paramIndex"
      ).mkString("ParamValue(", ", ", ")")
  }

  /**
    * 23 PARAM_SET
    *
    * Set a parameter value TEMPORARILY to RAM. It will be reset to default on system reboot. Send the ACTION MAV_ACTION_STORAGE_WRITE to PERMANENTLY write the RAM contents to EEPROM. IMPORTANT: The receiving component should acknowledge the new parameter value by sending a param_value message to all communication partners. This will also ensure that multiple GCS all have an up-to-date list of all parameters. If the sending GCS did not receive a PARAM_VALUE message within its timeout time, it should re-send the PARAM_SET message.
    *
    * @param targetSystem    System ID
    * @param targetComponent Component ID
    * @param paramId         Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
    * @param paramValue      Onboard parameter value
    * @param paramType       Onboard parameter type: see the MAV_PARAM_TYPE enum for supported data types.
    */
  case class ParamSet(
    targetSystem: Int,
    targetComponent: Int,
    paramId: String,
    paramValue: Float,
    paramType: MAVParamType
  ) extends MAVMessage("PARAM_SET", systemId, componentId, protocolVersion) {
    set("target_system", targetSystem)
    set("target_component", targetComponent)
    set("param_id", paramId)
    set("param_value", paramValue)
    set("param_type", paramType.value)

    override def toString: String =
      List(
        s"targetSystem = $targetSystem",
        s"targetComponent = $targetComponent",
        s"paramId = $paramId",
        s"paramValue = $paramValue",
        s"paramType = $paramType"
      ).mkString("ParamSet(", ", ", ")")
  }

  /**
    * 24 GPS_RAW_INT
    *
    * The global position, as returned by the Global Positioning System (GPS). This is
    * NOT the global position estimate of the system, but rather a RAW sensor value. See message GLOBAL_POSITION for the global position estimate. Coordinate frame is right-handed, Z-axis up (GPS frame).
    *
    * @param timeUSec          Timestamp (microseconds since UNIX epoch or microseconds since system boot)
    * @param fixType           See the GPS_FIX_TYPE enum.
    * @param lat               Latitude (WGS84, EGM96 ellipsoid), in degrees * 1E7
    * @param lon               Longitude (WGS84, EGM96 ellipsoid), in degrees * 1E7
    * @param alt               Altitude (AMSL, NOT WGS84), in meters * 1000 (positive for up). Note that virtually all GPS modules provide the AMSL altitude in addition to the WGS84 altitude.
    * @param epH               GPS HDOP horizontal dilution of position (unitless). If unknown, set to: UINT16_MAX
    * @param epV               GPS VDOP vertical dilution of position (unitless). If unknown, set to: UINT16_MAX
    * @param vel               GPS ground speed (m/s * 100). If unknown, set to: UINT16_MAX
    * @param cOG               Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
    * @param satellitesVisible Number of satellites visible. If unknown, set to 255
    * @param altEllipsoid      Altitude (above WGS84, EGM96 ellipsoid), in meters * 1000 (positive for up).
    * @param hAcc              Position uncertainty in meters * 1000 (positive for up).
    * @param vAcc              Altitude uncertainty in meters * 1000 (positive for up).
    * @param velAcc            Speed uncertainty in meters * 1000 (positive for up).
    * @param hdgAcc            Heading / track uncertainty in degrees * 1e5.
    */
  case class GPSRawInt(
    timeUSec: Long,
    fixType: GPSFixType,
    lat: Int,
    lon: Int,
    alt: Int,
    epH: Int,
    epV: Int,
    vel: Int,
    cOG: Int,
    satellitesVisible: Int,
    altEllipsoid: Int,
    hAcc: Int,
    vAcc: Int,
    velAcc: Int,
    hdgAcc: Int
  ) extends MAVMessage("GPS_RAW_INT", systemId, componentId, protocolVersion) {
    set("time_usec", timeUSec)
    set("fix_type", fixType.value)
    set("lat", lat)
    set("lon", lon)
    set("alt", alt)
    set("eph", epH)
    set("epv", epV)
    set("vel", vel)
    set("cog", cOG)
    set("satellites_visible", satellitesVisible)
    set("alt_ellipsoid", altEllipsoid)
    set("h_acc", hAcc)
    set("v_acc", vAcc)
    set("vel_acc", velAcc)
    set("hdg_acc", hdgAcc)

    override def toString: String =
      List(
        s"timeUSec = $timeUSec",
        s"fixType = $fixType",
        s"lat = $lat",
        s"lon = $lon",
        s"alt = $alt",
        s"epH = $epH",
        s"epV = $epV",
        s"vel = $vel",
        s"cOG = $cOG",
        s"satellitesVisible = $satellitesVisible",
        s"altEllipsoid = $altEllipsoid",
        s"hAcc = $hAcc",
        s"vAcc = $vAcc",
        s"velAcc = $velAcc",
        s"hdgAcc = $hdgAcc"
      ).mkString("GPSRawInt(", ", ", ")")
  }

  /**
    * 25 GPS_STATUS
    *
    * The positioning status, as reported by GPS. This message is intended to display status information about each satellite visible to the receiver. See message GLOBAL_POSITION for the global position estimate. This message can contain information for up to 20 satellites.
    *
    * @param satellitesVisible  Number of satellites visible
    * @param satellitePRN       Global satellite ID
    * @param satelliteUsed      0: Satellite not used, 1: used for localization
    * @param satelliteElevation Elevation (0: right on top of receiver, 90: on the horizon) of satellite
    * @param satelliteAzimuth   Direction of satellite, 0: 0 deg, 255: 360 deg.
    * @param satelliteSNR       Signal to noise ratio of satellite
    */
  case class GPSStatus(
    satellitesVisible: Int,
    satellitePRN: IndexedSeq[Int],
    satelliteUsed: IndexedSeq[Int],
    satelliteElevation: IndexedSeq[Int],
    satelliteAzimuth: IndexedSeq[Int],
    satelliteSNR: IndexedSeq[Int]
  ) extends MAVMessage("GPS_STATUS", systemId, componentId, protocolVersion) {
    set("satellites_visible", satellitesVisible)
    set("satellite_prn", satellitePRN.map(int2Integer).toArray)
    set("satellite_used", satelliteUsed.map(int2Integer).toArray)
    set("satellite_elevation", satelliteElevation.map(int2Integer).toArray)
    set("satellite_azimuth", satelliteAzimuth.map(int2Integer).toArray)
    set("satellite_snr", satelliteSNR.map(int2Integer).toArray)

    override def toString: String =
      List(
        s"satellitesVisible = $satellitesVisible",
        s"satellitePRN = $satellitePRN",
        s"satelliteUsed = $satelliteUsed",
        s"satelliteElevation = $satelliteElevation",
        s"satelliteAzimuth = $satelliteAzimuth",
        s"satelliteSNR = $satelliteSNR"
      ).mkString("GPSStatus(", ", ", ")")
  }

  /**
    * 26 SCALED_IMU
    *
    * The RAW IMU readings for the usual 9DOF sensor setup. This message should contain the scaled values to the described units
    *
    * @param timeBootMS Timestamp (milliseconds since system boot)
    * @param xAcc       X acceleration (mg)
    * @param yAcc       Y acceleration (mg)
    * @param zAcc       Z acceleration (mg)
    * @param xGyro      Angular speed around X axis (millirad /sec)
    * @param yGyro      Angular speed around Y axis (millirad /sec)
    * @param zGyro      Angular speed around Z axis (millirad /sec)
    * @param xMag       X Magnetic field (milli tesla)
    * @param yMag       Y Magnetic field (milli tesla)
    * @param zMag       Z Magnetic field (milli tesla)
    */
  case class ScaledIMU(
    timeBootMS: Int,
    xAcc: Int,
    yAcc: Int,
    zAcc: Int,
    xGyro: Int,
    yGyro: Int,
    zGyro: Int,
    xMag: Int,
    yMag: Int,
    zMag: Int
  ) extends MAVMessage("SCALED_IMU", systemId, componentId, protocolVersion) {
    set("time_boot_ms", timeBootMS)
    set("xacc", xAcc)
    set("yacc", yAcc)
    set("zacc", zAcc)
    set("xgyro", xGyro)
    set("ygyro", yGyro)
    set("zgyro", zGyro)
    set("xmag", xMag)
    set("ymag", yMag)
    set("zmag", zMag)

    override def toString: String =
      List(
        s"timeBootMS = $timeBootMS",
        s"xAcc = $xAcc",
        s"yAcc = $yAcc",
        s"zAcc = $zAcc",
        s"xGyro = $xGyro",
        s"yGyro = $yGyro",
        s"zGyro = $zGyro",
        s"xMag = $xMag",
        s"yMag = $yMag",
        s"zMag = $zMag"
      ).mkString("ScaledIMU(", ", ", ")")
  }

  /**
    * 27 RAW_IMU
    *
    * The RAW IMU readings for the usual 9DOF sensor setup. This message should always contain the true raw values without any scaling to allow data capture and system debugging.
    *
    * @param timeUSec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
    * @param xAcc     X acceleration (raw)
    * @param yAcc     Y acceleration (raw)
    * @param zAcc     Z acceleration (raw)
    * @param xGyro    Angular speed around X axis (raw)
    * @param yGyro    Angular speed around Y axis (raw)
    * @param zGyro    Angular speed around Z axis (raw)
    * @param xMag     X Magnetic field (raw)
    * @param yMag     Y Magnetic field (raw)
    * @param zMag     Z Magnetic field (raw)
    */
  case class RawIMU(
    timeUSec: Long,
    xAcc: Int,
    yAcc: Int,
    zAcc: Int,
    xGyro: Int,
    yGyro: Int,
    zGyro: Int,
    xMag: Int,
    yMag: Int,
    zMag: Int
  ) extends MAVMessage("RAW_IMU", systemId, componentId, protocolVersion) {
    set("time_usec", timeUSec)
    set("xacc", xAcc)
    set("yacc", yAcc)
    set("zacc", zAcc)
    set("xgyro", xGyro)
    set("ygyro", yGyro)
    set("zgyro", zGyro)
    set("xmag", xMag)
    set("ymag", yMag)
    set("zmag", zMag)

    override def toString: String =
      List(
        s"timeUSec = $timeUSec",
        s"xAcc = $xAcc",
        s"yAcc = $yAcc",
        s"zAcc = $zAcc",
        s"xGyro = $xGyro",
        s"yGyro = $yGyro",
        s"zGyro = $zGyro",
        s"xMag = $xMag",
        s"yMag = $yMag",
        s"zMag = $zMag"
      ).mkString("RawIMU(", ", ", ")")
  }

  /**
    * 28 RAW_PRESSURE
    *
    * The RAW pressure readings for the typical setup of one absolute pressure and one differential pressure sensor. The sensor values should be the raw, UNSCALED ADC values.
    *
    * @param timeUSec    Timestamp (microseconds since UNIX epoch or microseconds since system boot)
    * @param pressAbs    Absolute pressure (raw)
    * @param pressDiff1  Differential pressure 1 (raw, 0 if nonexistant)
    * @param pressDiff2  Differential pressure 2 (raw, 0 if nonexistant)
    * @param temperature Raw Temperature measurement (raw)
    */
  case class RawPressure(
    timeUSec: Long,
    pressAbs: Int,
    pressDiff1: Int,
    pressDiff2: Int,
    temperature: Int
  ) extends MAVMessage("RAW_PRESSURE", systemId, componentId, protocolVersion) {
    set("time_usec", timeUSec)
    set("press_abs", pressAbs)
    set("press_diff1", pressDiff1)
    set("press_diff2", pressDiff2)
    set("temperature", temperature)

    override def toString: String =
      List(
        s"timeUSec = $timeUSec",
        s"pressAbs = $pressAbs",
        s"pressDiff1 = $pressDiff1",
        s"pressDiff2 = $pressDiff2",
        s"temperature = $temperature"
      ).mkString("RawPressure(", ", ", ")")
  }

  /**
    * 29 SCALED_PRESSURE
    *
    * The pressure readings for the typical setup of one absolute and differential pressure sensor. The units are as specified in each field.
    *
    * @param timeBootMS  Timestamp (milliseconds since system boot)
    * @param pressAbs    Absolute pressure (hectopascal)
    * @param pressDiff   Differential pressure 1 (hectopascal)
    * @param temperature Temperature measurement (0.01 degrees celsius)
    */
  case class ScaledPressure(
    timeBootMS: Int,
    pressAbs: Float,
    pressDiff: Float,
    temperature: Int
  ) extends MAVMessage("SCALED_PRESSURE", systemId, componentId, protocolVersion) {
    set("time_boot_ms", timeBootMS)
    set("press_abs", pressAbs)
    set("press_diff", pressDiff)
    set("temperature", temperature)

    override def toString: String =
      List(
        s"timeBootMS = $timeBootMS",
        s"pressAbs = $pressAbs",
        s"pressDiff = $pressDiff",
        s"temperature = $temperature"
      ).mkString("ScaledPressure(", ", ", ")")
  }

  /**
    * 30 ATTITUDE
    *
    * The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right).
    *
    * @param timeBootMS Timestamp (milliseconds since system boot)
    * @param roll       Roll angle (rad, -pi..+pi)
    * @param pitch      Pitch angle (rad, -pi..+pi)
    * @param yaw        Yaw angle (rad, -pi..+pi)
    * @param rollSpeed  Roll angular speed (rad/s)
    * @param pitchSpeed Pitch angular speed (rad/s)
    * @param yawSpeed   Yaw angular speed (rad/s)
    */
  case class Attitude(
    timeBootMS: Int,
    roll: Float,
    pitch: Float,
    yaw: Float,
    rollSpeed: Float,
    pitchSpeed: Float,
    yawSpeed: Float
  ) extends MAVMessage("ATTITUDE", systemId, componentId, protocolVersion) {
    set("time_boot_ms", timeBootMS)
    set("roll", roll)
    set("pitch", pitch)
    set("yaw", yaw)
    set("rollspeed", rollSpeed)
    set("pitchspeed", pitchSpeed)
    set("yawspeed", yawSpeed)

    override def toString: String =
      List(
        s"timeBootMS = $timeBootMS",
        s"roll = $roll",
        s"pitch = $pitch",
        s"yaw = $yaw",
        s"rollSpeed = $rollSpeed",
        s"pitchSpeed = $pitchSpeed",
        s"yawSpeed = $yawSpeed"
      ).mkString("Attitude(", ", ", ")")
  }

  /**
    * 31 ATTITUDE_QUATERNION
    *
    * The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right), expressed as quaternion. Quaternion order is w, x, y, z and a zero rotation would be expressed as (1 0 0 0).
    *
    * @param timeBootMS Timestamp (milliseconds since system boot)
    * @param q1         Quaternion component 1, w (1 in null-rotation)
    * @param q2         Quaternion component 2, x (0 in null-rotation)
    * @param q3         Quaternion component 3, y (0 in null-rotation)
    * @param q4         Quaternion component 4, z (0 in null-rotation)
    * @param rollSpeed  Roll angular speed (rad/s)
    * @param pitchSpeed Pitch angular speed (rad/s)
    * @param yawSpeed   Yaw angular speed (rad/s)
    */
  case class AttitudeQuaternion(
    timeBootMS: Int,
    q1: Float,
    q2: Float,
    q3: Float,
    q4: Float,
    rollSpeed: Float,
    pitchSpeed: Float,
    yawSpeed: Float
  ) extends MAVMessage("ATTITUDE_QUATERNION", systemId, componentId, protocolVersion) {
    set("time_boot_ms", timeBootMS)
    set("q1", q1)
    set("q2", q2)
    set("q3", q3)
    set("q4", q4)
    set("rollspeed", rollSpeed)
    set("pitchspeed", pitchSpeed)
    set("yawspeed", yawSpeed)

    override def toString: String =
      List(
        s"timeBootMS = $timeBootMS",
        s"q1 = $q1",
        s"q2 = $q2",
        s"q3 = $q3",
        s"q4 = $q4",
        s"rollSpeed = $rollSpeed",
        s"pitchSpeed = $pitchSpeed",
        s"yawSpeed = $yawSpeed"
      ).mkString("AttitudeQuaternion(", ", ", ")")
  }

  /**
    * 32 LOCAL_POSITION_NED
    *
    * The filtered local position (e.g. fused computer vision and accelerometers). Coordinate frame is right-handed, Z-axis down (aeronautical frame, NED / north-east-down convention)
    *
    * @param timeBootMS Timestamp (milliseconds since system boot)
    * @param x          X Position
    * @param y          Y Position
    * @param z          Z Position
    * @param vX         X Speed
    * @param vY         Y Speed
    * @param vZ         Z Speed
    */
  case class LocalPositionNED(
    timeBootMS: Int,
    x: Float,
    y: Float,
    z: Float,
    vX: Float,
    vY: Float,
    vZ: Float
  ) extends MAVMessage("LOCAL_POSITION_NED", systemId, componentId, protocolVersion) {
    set("time_boot_ms", timeBootMS)
    set("x", x)
    set("y", y)
    set("z", z)
    set("vx", vX)
    set("vy", vY)
    set("vz", vZ)

    override def toString: String =
      List(
        s"timeBootMS = $timeBootMS",
        s"x = $x",
        s"y = $y",
        s"z = $z",
        s"vX = $vX",
        s"vY = $vY",
        s"vZ = $vZ"
      ).mkString("LocalPositionNED(", ", ", ")")
  }

  /**
    * 33 GLOBAL_POSITION_INT
    *
    * The filtered global position (e.g. fused GPS and accelerometers). The position is in GPS-frame (right-handed, Z-up). It
    * is designed as scaled integer message since the resolution of float is not sufficient.
    *
    * @param timeBootMS  Timestamp (milliseconds since system boot)
    * @param lat         Latitude, expressed as degrees * 1E7
    * @param lon         Longitude, expressed as degrees * 1E7
    * @param alt         Altitude in meters, expressed as * 1000 (millimeters), AMSL (not WGS84 - note that virtually all GPS modules provide the AMSL as well)
    * @param relativeAlt Altitude above ground in meters, expressed as * 1000 (millimeters)
    * @param vX          Ground X Speed (Latitude, positive north), expressed as m/s * 100
    * @param vY          Ground Y Speed (Longitude, positive east), expressed as m/s * 100
    * @param vZ          Ground Z Speed (Altitude, positive down), expressed as m/s * 100
    * @param hdg         Vehicle heading (yaw angle) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
    */
  case class GlobalPositionInt(
    timeBootMS: Int,
    lat: Int,
    lon: Int,
    alt: Int,
    relativeAlt: Int,
    vX: Int,
    vY: Int,
    vZ: Int,
    hdg: Int
  ) extends MAVMessage("GLOBAL_POSITION_INT", systemId, componentId, protocolVersion) {
    set("time_boot_ms", timeBootMS)
    set("lat", lat)
    set("lon", lon)
    set("alt", alt)
    set("relative_alt", relativeAlt)
    set("vx", vX)
    set("vy", vY)
    set("vz", vZ)
    set("hdg", hdg)

    override def toString: String =
      List(
        s"timeBootMS = $timeBootMS",
        s"lat = $lat",
        s"lon = $lon",
        s"alt = $alt",
        s"relativeAlt = $relativeAlt",
        s"vX = $vX",
        s"vY = $vY",
        s"vZ = $vZ",
        s"hdg = $hdg"
      ).mkString("GlobalPositionInt(", ", ", ")")
  }

  /**
    * 34 RC_CHANNELS_SCALED
    *
    * The scaled values of the RC channels received. (-100%) -10000, (0%) 0, (100%) 10000. Channels that are inactive should be set to UINT16_MAX.
    *
    * @param timeBootMS  Timestamp (milliseconds since system boot)
    * @param port        Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows for more than 8 servos.
    * @param chan1Scaled RC channel 1 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
    * @param chan2Scaled RC channel 2 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
    * @param chan3Scaled RC channel 3 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
    * @param chan4Scaled RC channel 4 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
    * @param chan5Scaled RC channel 5 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
    * @param chan6Scaled RC channel 6 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
    * @param chan7Scaled RC channel 7 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
    * @param chan8Scaled RC channel 8 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
    * @param rSSI        Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown.
    */
  case class RCChannelsScaled(
    timeBootMS: Int,
    port: Int,
    chan1Scaled: Int,
    chan2Scaled: Int,
    chan3Scaled: Int,
    chan4Scaled: Int,
    chan5Scaled: Int,
    chan6Scaled: Int,
    chan7Scaled: Int,
    chan8Scaled: Int,
    rSSI: Int
  ) extends MAVMessage("RC_CHANNELS_SCALED", systemId, componentId, protocolVersion) {
    set("time_boot_ms", timeBootMS)
    set("port", port)
    set("chan1_scaled", chan1Scaled)
    set("chan2_scaled", chan2Scaled)
    set("chan3_scaled", chan3Scaled)
    set("chan4_scaled", chan4Scaled)
    set("chan5_scaled", chan5Scaled)
    set("chan6_scaled", chan6Scaled)
    set("chan7_scaled", chan7Scaled)
    set("chan8_scaled", chan8Scaled)
    set("rssi", rSSI)

    override def toString: String =
      List(
        s"timeBootMS = $timeBootMS",
        s"port = $port",
        s"chan1Scaled = $chan1Scaled",
        s"chan2Scaled = $chan2Scaled",
        s"chan3Scaled = $chan3Scaled",
        s"chan4Scaled = $chan4Scaled",
        s"chan5Scaled = $chan5Scaled",
        s"chan6Scaled = $chan6Scaled",
        s"chan7Scaled = $chan7Scaled",
        s"chan8Scaled = $chan8Scaled",
        s"rSSI = $rSSI"
      ).mkString("RCChannelsScaled(", ", ", ")")
  }

  /**
    * 35 RC_CHANNELS_RAW
    *
    * The RAW values of the RC channels received. The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%. Individual receivers/transmitters might violate this specification.
    *
    * @param timeBootMS Timestamp (milliseconds since system boot)
    * @param port       Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows for more than 8 servos.
    * @param chan1Raw   RC channel 1 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
    * @param chan2Raw   RC channel 2 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
    * @param chan3Raw   RC channel 3 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
    * @param chan4Raw   RC channel 4 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
    * @param chan5Raw   RC channel 5 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
    * @param chan6Raw   RC channel 6 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
    * @param chan7Raw   RC channel 7 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
    * @param chan8Raw   RC channel 8 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
    * @param rSSI       Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown.
    */
  case class RCChannelsRaw(
    timeBootMS: Int,
    port: Int,
    chan1Raw: Int,
    chan2Raw: Int,
    chan3Raw: Int,
    chan4Raw: Int,
    chan5Raw: Int,
    chan6Raw: Int,
    chan7Raw: Int,
    chan8Raw: Int,
    rSSI: Int
  ) extends MAVMessage("RC_CHANNELS_RAW", systemId, componentId, protocolVersion) {
    set("time_boot_ms", timeBootMS)
    set("port", port)
    set("chan1_raw", chan1Raw)
    set("chan2_raw", chan2Raw)
    set("chan3_raw", chan3Raw)
    set("chan4_raw", chan4Raw)
    set("chan5_raw", chan5Raw)
    set("chan6_raw", chan6Raw)
    set("chan7_raw", chan7Raw)
    set("chan8_raw", chan8Raw)
    set("rssi", rSSI)

    override def toString: String =
      List(
        s"timeBootMS = $timeBootMS",
        s"port = $port",
        s"chan1Raw = $chan1Raw",
        s"chan2Raw = $chan2Raw",
        s"chan3Raw = $chan3Raw",
        s"chan4Raw = $chan4Raw",
        s"chan5Raw = $chan5Raw",
        s"chan6Raw = $chan6Raw",
        s"chan7Raw = $chan7Raw",
        s"chan8Raw = $chan8Raw",
        s"rSSI = $rSSI"
      ).mkString("RCChannelsRaw(", ", ", ")")
  }

  /**
    * 36 SERVO_OUTPUT_RAW
    *
    * The RAW values of the servo outputs (for RC input from the remote, use the RC_CHANNELS messages). The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%.
    *
    * @param timeUSec   Timestamp (microseconds since system boot)
    * @param port       Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows to encode more than 8 servos.
    * @param servo1Raw  Servo output 1 value, in microseconds
    * @param servo2Raw  Servo output 2 value, in microseconds
    * @param servo3Raw  Servo output 3 value, in microseconds
    * @param servo4Raw  Servo output 4 value, in microseconds
    * @param servo5Raw  Servo output 5 value, in microseconds
    * @param servo6Raw  Servo output 6 value, in microseconds
    * @param servo7Raw  Servo output 7 value, in microseconds
    * @param servo8Raw  Servo output 8 value, in microseconds
    * @param servo9Raw  Servo output 9 value, in microseconds
    * @param servo10Raw Servo output 10 value, in microseconds
    * @param servo11Raw Servo output 11 value, in microseconds
    * @param servo12Raw Servo output 12 value, in microseconds
    * @param servo13Raw Servo output 13 value, in microseconds
    * @param servo14Raw Servo output 14 value, in microseconds
    * @param servo15Raw Servo output 15 value, in microseconds
    * @param servo16Raw Servo output 16 value, in microseconds
    */
  case class ServoOutputRaw(
    timeUSec: Int,
    port: Int,
    servo1Raw: Int,
    servo2Raw: Int,
    servo3Raw: Int,
    servo4Raw: Int,
    servo5Raw: Int,
    servo6Raw: Int,
    servo7Raw: Int,
    servo8Raw: Int,
    servo9Raw: Int,
    servo10Raw: Int,
    servo11Raw: Int,
    servo12Raw: Int,
    servo13Raw: Int,
    servo14Raw: Int,
    servo15Raw: Int,
    servo16Raw: Int
  ) extends MAVMessage("SERVO_OUTPUT_RAW", systemId, componentId, protocolVersion) {
    set("time_usec", timeUSec)
    set("port", port)
    set("servo1_raw", servo1Raw)
    set("servo2_raw", servo2Raw)
    set("servo3_raw", servo3Raw)
    set("servo4_raw", servo4Raw)
    set("servo5_raw", servo5Raw)
    set("servo6_raw", servo6Raw)
    set("servo7_raw", servo7Raw)
    set("servo8_raw", servo8Raw)
    set("servo9_raw", servo9Raw)
    set("servo10_raw", servo10Raw)
    set("servo11_raw", servo11Raw)
    set("servo12_raw", servo12Raw)
    set("servo13_raw", servo13Raw)
    set("servo14_raw", servo14Raw)
    set("servo15_raw", servo15Raw)
    set("servo16_raw", servo16Raw)

    override def toString: String =
      List(
        s"timeUSec = $timeUSec",
        s"port = $port",
        s"servo1Raw = $servo1Raw",
        s"servo2Raw = $servo2Raw",
        s"servo3Raw = $servo3Raw",
        s"servo4Raw = $servo4Raw",
        s"servo5Raw = $servo5Raw",
        s"servo6Raw = $servo6Raw",
        s"servo7Raw = $servo7Raw",
        s"servo8Raw = $servo8Raw",
        s"servo9Raw = $servo9Raw",
        s"servo10Raw = $servo10Raw",
        s"servo11Raw = $servo11Raw",
        s"servo12Raw = $servo12Raw",
        s"servo13Raw = $servo13Raw",
        s"servo14Raw = $servo14Raw",
        s"servo15Raw = $servo15Raw",
        s"servo16Raw = $servo16Raw"
      ).mkString("ServoOutputRaw(", ", ", ")")
  }

  /**
    * 37 MISSION_REQUEST_PARTIAL_LIST
    *
    * Request a partial list of mission items from the system/component. http://qgroundcontrol.org/mavlink/waypoint_protocol. If start and end index are the same, just send one waypoint.
    *
    * @param targetSystem    System ID
    * @param targetComponent Component ID
    * @param startIndex      Start index, 0 by default
    * @param endIndex        End index, -1 by default (-1: send list to end). Else a valid index of the list
    * @param missionType     Mission type, see MAV_MISSION_TYPE
    */
  case class MissionRequestPartialList(
    targetSystem: Int,
    targetComponent: Int,
    startIndex: Int,
    endIndex: Int,
    missionType: MAVMissionType
  ) extends MAVMessage("MISSION_REQUEST_PARTIAL_LIST", systemId, componentId, protocolVersion) {
    set("target_system", targetSystem)
    set("target_component", targetComponent)
    set("start_index", startIndex)
    set("end_index", endIndex)
    set("mission_type", missionType.value)

    override def toString: String =
      List(
        s"targetSystem = $targetSystem",
        s"targetComponent = $targetComponent",
        s"startIndex = $startIndex",
        s"endIndex = $endIndex",
        s"missionType = $missionType"
      ).mkString("MissionRequestPartialList(", ", ", ")")
  }

  /**
    * 38 MISSION_WRITE_PARTIAL_LIST
    *
    * This message is sent to the MAV to write a partial list. If start index == end index, only one item will be transmitted / updated. If the start index is NOT 0 and above the current list size, this request should be REJECTED!
    *
    * @param targetSystem    System ID
    * @param targetComponent Component ID
    * @param startIndex      Start index, 0 by default and smaller / equal to the largest index of the current onboard list.
    * @param endIndex        End index, equal or greater than start index.
    * @param missionType     Mission type, see MAV_MISSION_TYPE
    */
  case class MissionWritePartialList(
    targetSystem: Int,
    targetComponent: Int,
    startIndex: Int,
    endIndex: Int,
    missionType: MAVMissionType
  ) extends MAVMessage("MISSION_WRITE_PARTIAL_LIST", systemId, componentId, protocolVersion) {
    set("target_system", targetSystem)
    set("target_component", targetComponent)
    set("start_index", startIndex)
    set("end_index", endIndex)
    set("mission_type", missionType.value)

    override def toString: String =
      List(
        s"targetSystem = $targetSystem",
        s"targetComponent = $targetComponent",
        s"startIndex = $startIndex",
        s"endIndex = $endIndex",
        s"missionType = $missionType"
      ).mkString("MissionWritePartialList(", ", ", ")")
  }

  /**
    * 39 MISSION_ITEM
    *
    * Message encoding a mission item. This message is emitted to announce
    * the presence of a mission item and to set a mission item on the system. The mission item can be either in x, y, z meters (type: LOCAL) or x:lat, y:lon, z:altitude. Local frame is Z-down, right handed (NED), global frame is Z-up, right handed (ENU). See also http://qgroundcontrol.org/mavlink/waypoint_protocol.
    *
    * @param targetSystem    System ID
    * @param targetComponent Component ID
    * @param seq             Sequence
    * @param frame           The coordinate system of the waypoint. see MAV_FRAME in mavlink_types.h
    * @param command         The scheduled action for the waypoint. see MAV_CMD in common.xml MAVLink specs
    * @param current         false:0, true:1
    * @param autoContinue    autocontinue to next wp
    * @param param1          PARAM1, see MAV_CMD enum
    * @param param2          PARAM2, see MAV_CMD enum
    * @param param3          PARAM3, see MAV_CMD enum
    * @param param4          PARAM4, see MAV_CMD enum
    * @param x               PARAM5 / local: x position, global: latitude
    * @param y               PARAM6 / y position: global: longitude
    * @param z               PARAM7 / z position: global: altitude (relative or absolute, depending on frame.
    * @param missionType     Mission type, see MAV_MISSION_TYPE
    */
  case class MissionItem(
    targetSystem: Int,
    targetComponent: Int,
    seq: Int,
    frame: MAVFrame,
    command: MAVCmd,
    current: Int,
    autoContinue: Int,
    param1: Float,
    param2: Float,
    param3: Float,
    param4: Float,
    x: Float,
    y: Float,
    z: Float,
    missionType: MAVMissionType
  ) extends MAVMessage("MISSION_ITEM", systemId, componentId, protocolVersion) {
    set("target_system", targetSystem)
    set("target_component", targetComponent)
    set("seq", seq)
    set("frame", frame.value)
    set("command", command.value)
    set("current", current)
    set("autocontinue", autoContinue)
    set("param1", param1)
    set("param2", param2)
    set("param3", param3)
    set("param4", param4)
    set("x", x)
    set("y", y)
    set("z", z)
    set("mission_type", missionType.value)

    override def toString: String =
      List(
        s"targetSystem = $targetSystem",
        s"targetComponent = $targetComponent",
        s"seq = $seq",
        s"frame = $frame",
        s"command = $command",
        s"current = $current",
        s"autoContinue = $autoContinue",
        s"param1 = $param1",
        s"param2 = $param2",
        s"param3 = $param3",
        s"param4 = $param4",
        s"x = $x",
        s"y = $y",
        s"z = $z",
        s"missionType = $missionType"
      ).mkString("MissionItem(", ", ", ")")
  }

  /**
    * 40 MISSION_REQUEST
    *
    * Request the information of the mission item with the sequence number seq. The response of the system to this message should be a MISSION_ITEM message. http://qgroundcontrol.org/mavlink/waypoint_protocol
    *
    * @param targetSystem    System ID
    * @param targetComponent Component ID
    * @param seq             Sequence
    * @param missionType     Mission type, see MAV_MISSION_TYPE
    */
  case class MissionRequest(
    targetSystem: Int,
    targetComponent: Int,
    seq: Int,
    missionType: MAVMissionType
  ) extends MAVMessage("MISSION_REQUEST", systemId, componentId, protocolVersion) {
    set("target_system", targetSystem)
    set("target_component", targetComponent)
    set("seq", seq)
    set("mission_type", missionType.value)

    override def toString: String =
      List(
        s"targetSystem = $targetSystem",
        s"targetComponent = $targetComponent",
        s"seq = $seq",
        s"missionType = $missionType"
      ).mkString("MissionRequest(", ", ", ")")
  }

  /**
    * 41 MISSION_SET_CURRENT
    *
    * Set the mission item with sequence number seq as current item. This means that the MAV will continue to this mission item on the shortest path (not following the mission items in-between).
    *
    * @param targetSystem    System ID
    * @param targetComponent Component ID
    * @param seq             Sequence
    */
  case class MissionSetCurrent(
    targetSystem: Int,
    targetComponent: Int,
    seq: Int
  ) extends MAVMessage("MISSION_SET_CURRENT", systemId, componentId, protocolVersion) {
    set("target_system", targetSystem)
    set("target_component", targetComponent)
    set("seq", seq)

    override def toString: String =
      List(
        s"targetSystem = $targetSystem",
        s"targetComponent = $targetComponent",
        s"seq = $seq"
      ).mkString("MissionSetCurrent(", ", ", ")")
  }

  /**
    * 42 MISSION_CURRENT
    *
    * Message that announces the sequence number of the current active mission item. The MAV will fly towards this mission item.
    *
    * @param seq Sequence
    */
  case class MissionCurrent(
    seq: Int
  ) extends MAVMessage("MISSION_CURRENT", systemId, componentId, protocolVersion) {
    set("seq", seq)

    override def toString: String =
      List(
        s"seq = $seq"
      ).mkString("MissionCurrent(", ", ", ")")
  }

  /**
    * 43 MISSION_REQUEST_LIST
    *
    * Request the overall list of mission items from the system/component.
    *
    * @param targetSystem    System ID
    * @param targetComponent Component ID
    * @param missionType     Mission type, see MAV_MISSION_TYPE
    */
  case class MissionRequestList(
    targetSystem: Int,
    targetComponent: Int,
    missionType: MAVMissionType
  ) extends MAVMessage("MISSION_REQUEST_LIST", systemId, componentId, protocolVersion) {
    set("target_system", targetSystem)
    set("target_component", targetComponent)
    set("mission_type", missionType.value)

    override def toString: String =
      List(
        s"targetSystem = $targetSystem",
        s"targetComponent = $targetComponent",
        s"missionType = $missionType"
      ).mkString("MissionRequestList(", ", ", ")")
  }

  /**
    * 44 MISSION_COUNT
    *
    * This message is emitted as response to MISSION_REQUEST_LIST by the MAV and to initiate a write transaction. The GCS can then request the individual mission item based on the knowledge of the total number of waypoints.
    *
    * @param targetSystem    System ID
    * @param targetComponent Component ID
    * @param count           Number of mission items in the sequence
    * @param missionType     Mission type, see MAV_MISSION_TYPE
    */
  case class MissionCount(
    targetSystem: Int,
    targetComponent: Int,
    count: Int,
    missionType: MAVMissionType
  ) extends MAVMessage("MISSION_COUNT", systemId, componentId, protocolVersion) {
    set("target_system", targetSystem)
    set("target_component", targetComponent)
    set("count", count)
    set("mission_type", missionType.value)

    override def toString: String =
      List(
        s"targetSystem = $targetSystem",
        s"targetComponent = $targetComponent",
        s"count = $count",
        s"missionType = $missionType"
      ).mkString("MissionCount(", ", ", ")")
  }

  /**
    * 45 MISSION_CLEAR_ALL
    *
    * Delete all mission items at once.
    *
    * @param targetSystem    System ID
    * @param targetComponent Component ID
    * @param missionType     Mission type, see MAV_MISSION_TYPE
    */
  case class MissionClearAll(
    targetSystem: Int,
    targetComponent: Int,
    missionType: MAVMissionType
  ) extends MAVMessage("MISSION_CLEAR_ALL", systemId, componentId, protocolVersion) {
    set("target_system", targetSystem)
    set("target_component", targetComponent)
    set("mission_type", missionType.value)

    override def toString: String =
      List(
        s"targetSystem = $targetSystem",
        s"targetComponent = $targetComponent",
        s"missionType = $missionType"
      ).mkString("MissionClearAll(", ", ", ")")
  }

  /**
    * 46 MISSION_ITEM_REACHED
    *
    * A certain mission item has been reached. The system will either hold this position (or circle on the orbit) or (if the autocontinue on the WP was set) continue to the next waypoint.
    *
    * @param seq Sequence
    */
  case class MissionItemReached(
    seq: Int
  ) extends MAVMessage("MISSION_ITEM_REACHED", systemId, componentId, protocolVersion) {
    set("seq", seq)

    override def toString: String =
      List(
        s"seq = $seq"
      ).mkString("MissionItemReached(", ", ", ")")
  }

  /**
    * 47 MISSION_ACK
    *
    * Ack message during waypoint handling. The type field states if this message is a positive ack (type=0) or if an error happened (type=non-zero).
    *
    * @param targetSystem    System ID
    * @param targetComponent Component ID
    * @param `type`          See MAV_MISSION_RESULT enum
    * @param missionType     Mission type, see MAV_MISSION_TYPE
    */
  case class MissionAck(
    targetSystem: Int,
    targetComponent: Int,
    `type`: MAVMissionResult,
    missionType: MAVMissionType
  ) extends MAVMessage("MISSION_ACK", systemId, componentId, protocolVersion) {
    set("target_system", targetSystem)
    set("target_component", targetComponent)
    set("type", `type`.value)
    set("mission_type", missionType.value)

    override def toString: String =
      List(
        s"targetSystem = $targetSystem",
        s"targetComponent = $targetComponent",
        s"type = ${`type`}",
        s"missionType = $missionType"
      ).mkString("MissionAck(", ", ", ")")
  }

  /**
    * 48 SET_GPS_GLOBAL_ORIGIN
    *
    * As local waypoints exist, the global waypoint reference allows to transform between the local coordinate frame and the global (GPS) coordinate frame. This can be necessary when e.g. in- and outdoor settings are connected and the MAV should move from in- to outdoor.
    *
    * @param targetSystem System ID
    * @param latitude     Latitude (WGS84), in degrees * 1E7
    * @param longitude    Longitude (WGS84, in degrees * 1E7
    * @param altitude     Altitude (AMSL), in meters * 1000 (positive for up)
    * @param timeUSec     Timestamp (microseconds since UNIX epoch or microseconds since system boot)
    */
  case class SetGPSGlobalOrigin(
    targetSystem: Int,
    latitude: Int,
    longitude: Int,
    altitude: Int,
    timeUSec: Long
  ) extends MAVMessage("SET_GPS_GLOBAL_ORIGIN", systemId, componentId, protocolVersion) {
    set("target_system", targetSystem)
    set("latitude", latitude)
    set("longitude", longitude)
    set("altitude", altitude)
    set("time_usec", timeUSec)

    override def toString: String =
      List(
        s"targetSystem = $targetSystem",
        s"latitude = $latitude",
        s"longitude = $longitude",
        s"altitude = $altitude",
        s"timeUSec = $timeUSec"
      ).mkString("SetGPSGlobalOrigin(", ", ", ")")
  }

  /**
    * 49 GPS_GLOBAL_ORIGIN
    *
    * Once the MAV sets a new GPS-Local correspondence, this message announces the origin (0,0,0) position
    *
    * @param latitude  Latitude (WGS84), in degrees * 1E7
    * @param longitude Longitude (WGS84), in degrees * 1E7
    * @param altitude  Altitude (AMSL), in meters * 1000 (positive for up)
    * @param timeUSec  Timestamp (microseconds since UNIX epoch or microseconds since system boot)
    */
  case class GPSGlobalOrigin(
    latitude: Int,
    longitude: Int,
    altitude: Int,
    timeUSec: Long
  ) extends MAVMessage("GPS_GLOBAL_ORIGIN", systemId, componentId, protocolVersion) {
    set("latitude", latitude)
    set("longitude", longitude)
    set("altitude", altitude)
    set("time_usec", timeUSec)

    override def toString: String =
      List(
        s"latitude = $latitude",
        s"longitude = $longitude",
        s"altitude = $altitude",
        s"timeUSec = $timeUSec"
      ).mkString("GPSGlobalOrigin(", ", ", ")")
  }

  /**
    * 50 PARAM_MAP_RC
    *
    * Bind a RC channel to a parameter. The parameter should change accoding to the RC channel value.
    *
    * @param targetSystem            System ID
    * @param targetComponent         Component ID
    * @param paramId                 Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
    * @param paramIndex              Parameter index. Send -1 to use the param ID field as identifier (else the param id will be ignored), send -2 to disable any existing map for this rc_channel_index.
    * @param parameterRCChannelIndex Index of parameter RC channel. Not equal to the RC channel id. Typically correpsonds to a potentiometer-knob on the RC.
    * @param paramValue0             Initial parameter value
    * @param scale                   Scale, maps the RC range [-1, 1] to a parameter value
    * @param paramValueMin           Minimum param value. The protocol does not define if this overwrites an onboard minimum value. (Depends on implementation)
    * @param paramValueMax           Maximum param value. The protocol does not define if this overwrites an onboard maximum value. (Depends on implementation)
    */
  case class ParamMapRC(
    targetSystem: Int,
    targetComponent: Int,
    paramId: String,
    paramIndex: Int,
    parameterRCChannelIndex: Int,
    paramValue0: Float,
    scale: Float,
    paramValueMin: Float,
    paramValueMax: Float
  ) extends MAVMessage("PARAM_MAP_RC", systemId, componentId, protocolVersion) {
    set("target_system", targetSystem)
    set("target_component", targetComponent)
    set("param_id", paramId)
    set("param_index", paramIndex)
    set("parameter_rc_channel_index", parameterRCChannelIndex)
    set("param_value0", paramValue0)
    set("scale", scale)
    set("param_value_min", paramValueMin)
    set("param_value_max", paramValueMax)

    override def toString: String =
      List(
        s"targetSystem = $targetSystem",
        s"targetComponent = $targetComponent",
        s"paramId = $paramId",
        s"paramIndex = $paramIndex",
        s"parameterRCChannelIndex = $parameterRCChannelIndex",
        s"paramValue0 = $paramValue0",
        s"scale = $scale",
        s"paramValueMin = $paramValueMin",
        s"paramValueMax = $paramValueMax"
      ).mkString("ParamMapRC(", ", ", ")")
  }

  /**
    * 51 MISSION_REQUEST_INT
    *
    * Request the information of the mission item with the sequence number seq. The response of the system to this message should be a MISSION_ITEM_INT message. http://qgroundcontrol.org/mavlink/waypoint_protocol
    *
    * @param targetSystem    System ID
    * @param targetComponent Component ID
    * @param seq             Sequence
    * @param missionType     Mission type, see MAV_MISSION_TYPE
    */
  case class MissionRequestInt(
    targetSystem: Int,
    targetComponent: Int,
    seq: Int,
    missionType: MAVMissionType
  ) extends MAVMessage("MISSION_REQUEST_INT", systemId, componentId, protocolVersion) {
    set("target_system", targetSystem)
    set("target_component", targetComponent)
    set("seq", seq)
    set("mission_type", missionType.value)

    override def toString: String =
      List(
        s"targetSystem = $targetSystem",
        s"targetComponent = $targetComponent",
        s"seq = $seq",
        s"missionType = $missionType"
      ).mkString("MissionRequestInt(", ", ", ")")
  }

  /**
    * 54 SAFETY_SET_ALLOWED_AREA
    *
    * Set a safety zone (volume), which is defined by two corners of a cube. This message can be used to tell the MAV which setpoints/waypoints to accept and which to reject. Safety areas are often enforced by national or competition regulations.
    *
    * @param targetSystem    System ID
    * @param targetComponent Component ID
    * @param frame           Coordinate frame, as defined by MAV_FRAME enum in mavlink_types.h. Can be either global, GPS, right-handed with Z axis up or local, right handed, Z axis down.
    * @param p1X             x position 1 / Latitude 1
    * @param p1Y             y position 1 / Longitude 1
    * @param p1Z             z position 1 / Altitude 1
    * @param p2X             x position 2 / Latitude 2
    * @param p2Y             y position 2 / Longitude 2
    * @param p2Z             z position 2 / Altitude 2
    */
  case class SafetySetAllowedArea(
    targetSystem: Int,
    targetComponent: Int,
    frame: MAVFrame,
    p1X: Float,
    p1Y: Float,
    p1Z: Float,
    p2X: Float,
    p2Y: Float,
    p2Z: Float
  ) extends MAVMessage("SAFETY_SET_ALLOWED_AREA", systemId, componentId, protocolVersion) {
    set("target_system", targetSystem)
    set("target_component", targetComponent)
    set("frame", frame.value)
    set("p1x", p1X)
    set("p1y", p1Y)
    set("p1z", p1Z)
    set("p2x", p2X)
    set("p2y", p2Y)
    set("p2z", p2Z)

    override def toString: String =
      List(
        s"targetSystem = $targetSystem",
        s"targetComponent = $targetComponent",
        s"frame = $frame",
        s"p1X = $p1X",
        s"p1Y = $p1Y",
        s"p1Z = $p1Z",
        s"p2X = $p2X",
        s"p2Y = $p2Y",
        s"p2Z = $p2Z"
      ).mkString("SafetySetAllowedArea(", ", ", ")")
  }

  /**
    * 55 SAFETY_ALLOWED_AREA
    *
    * Read out the safety zone the MAV currently assumes.
    *
    * @param frame Coordinate frame, as defined by MAV_FRAME enum in mavlink_types.h. Can be either global, GPS, right-handed with Z axis up or local, right handed, Z axis down.
    * @param p1X   x position 1 / Latitude 1
    * @param p1Y   y position 1 / Longitude 1
    * @param p1Z   z position 1 / Altitude 1
    * @param p2X   x position 2 / Latitude 2
    * @param p2Y   y position 2 / Longitude 2
    * @param p2Z   z position 2 / Altitude 2
    */
  case class SafetyAllowedArea(
    frame: MAVFrame,
    p1X: Float,
    p1Y: Float,
    p1Z: Float,
    p2X: Float,
    p2Y: Float,
    p2Z: Float
  ) extends MAVMessage("SAFETY_ALLOWED_AREA", systemId, componentId, protocolVersion) {
    set("frame", frame.value)
    set("p1x", p1X)
    set("p1y", p1Y)
    set("p1z", p1Z)
    set("p2x", p2X)
    set("p2y", p2Y)
    set("p2z", p2Z)

    override def toString: String =
      List(
        s"frame = $frame",
        s"p1X = $p1X",
        s"p1Y = $p1Y",
        s"p1Z = $p1Z",
        s"p2X = $p2X",
        s"p2Y = $p2Y",
        s"p2Z = $p2Z"
      ).mkString("SafetyAllowedArea(", ", ", ")")
  }

  /**
    * 61 ATTITUDE_QUATERNION_COV
    *
    * The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right), expressed as quaternion. Quaternion order is w, x, y, z and a zero rotation would be expressed as (1 0 0 0).
    *
    * @param timeUSec   Timestamp (microseconds since system boot or since UNIX epoch)
    * @param q          Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation)
    * @param rollSpeed  Roll angular speed (rad/s)
    * @param pitchSpeed Pitch angular speed (rad/s)
    * @param yawSpeed   Yaw angular speed (rad/s)
    * @param covariance Attitude covariance
    */
  case class AttitudeQuaternionCov(
    timeUSec: Long,
    q: IndexedSeq[Float],
    rollSpeed: Float,
    pitchSpeed: Float,
    yawSpeed: Float,
    covariance: IndexedSeq[Float]
  ) extends MAVMessage("ATTITUDE_QUATERNION_COV", systemId, componentId, protocolVersion) {
    set("time_usec", timeUSec)
    set("q", q.map(float2Float).toArray)
    set("rollspeed", rollSpeed)
    set("pitchspeed", pitchSpeed)
    set("yawspeed", yawSpeed)
    set("covariance", covariance.map(float2Float).toArray)

    override def toString: String =
      List(
        s"timeUSec = $timeUSec",
        s"q = $q",
        s"rollSpeed = $rollSpeed",
        s"pitchSpeed = $pitchSpeed",
        s"yawSpeed = $yawSpeed",
        s"covariance = $covariance"
      ).mkString("AttitudeQuaternionCov(", ", ", ")")
  }

  /**
    * 62 NAV_CONTROLLER_OUTPUT
    *
    * The state of the fixed wing navigation and position controller.
    *
    * @param navRoll       Current desired roll in degrees
    * @param navPitch      Current desired pitch in degrees
    * @param navBearing    Current desired heading in degrees
    * @param targetBearing Bearing to current waypoint/target in degrees
    * @param wpDist        Distance to active waypoint in meters
    * @param altError      Current altitude error in meters
    * @param aSPDError     Current airspeed error in meters/second
    * @param xTrackError   Current crosstrack error on x-y plane in meters
    */
  case class NavControllerOutput(
    navRoll: Float,
    navPitch: Float,
    navBearing: Int,
    targetBearing: Int,
    wpDist: Int,
    altError: Float,
    aSPDError: Float,
    xTrackError: Float
  ) extends MAVMessage("NAV_CONTROLLER_OUTPUT", systemId, componentId, protocolVersion) {
    set("nav_roll", navRoll)
    set("nav_pitch", navPitch)
    set("nav_bearing", navBearing)
    set("target_bearing", targetBearing)
    set("wp_dist", wpDist)
    set("alt_error", altError)
    set("aspd_error", aSPDError)
    set("xtrack_error", xTrackError)

    override def toString: String =
      List(
        s"navRoll = $navRoll",
        s"navPitch = $navPitch",
        s"navBearing = $navBearing",
        s"targetBearing = $targetBearing",
        s"wpDist = $wpDist",
        s"altError = $altError",
        s"aSPDError = $aSPDError",
        s"xTrackError = $xTrackError"
      ).mkString("NavControllerOutput(", ", ", ")")
  }

  /**
    * 63 GLOBAL_POSITION_INT_COV
    *
    * The filtered global position (e.g. fused GPS and accelerometers). The position is in GPS-frame (right-handed, Z-up). It  is designed as scaled integer message since the resolution of float is not sufficient. NOTE: This message is intended for onboard networks / companion computers and higher-bandwidth links and optimized for accuracy and completeness. Please use the GLOBAL_POSITION_INT message for a minimal subset.
    *
    * @param timeUSec      Timestamp (microseconds since system boot or since UNIX epoch)
    * @param estimatorType Class id of the estimator this estimate originated from.
    * @param lat           Latitude, expressed as degrees * 1E7
    * @param lon           Longitude, expressed as degrees * 1E7
    * @param alt           Altitude in meters, expressed as * 1000 (millimeters), above MSL
    * @param relativeAlt   Altitude above ground in meters, expressed as * 1000 (millimeters)
    * @param vX            Ground X Speed (Latitude), expressed as m/s
    * @param vY            Ground Y Speed (Longitude), expressed as m/s
    * @param vZ            Ground Z Speed (Altitude), expressed as m/s
    * @param covariance    Covariance matrix (first six entries are the first ROW, next six entries are the second row, etc.)
    */
  case class GlobalPositionIntCov(
    timeUSec: Long,
    estimatorType: MAVEstimatorType,
    lat: Int,
    lon: Int,
    alt: Int,
    relativeAlt: Int,
    vX: Float,
    vY: Float,
    vZ: Float,
    covariance: IndexedSeq[Float]
  ) extends MAVMessage("GLOBAL_POSITION_INT_COV", systemId, componentId, protocolVersion) {
    set("time_usec", timeUSec)
    set("estimator_type", estimatorType.value)
    set("lat", lat)
    set("lon", lon)
    set("alt", alt)
    set("relative_alt", relativeAlt)
    set("vx", vX)
    set("vy", vY)
    set("vz", vZ)
    set("covariance", covariance.map(float2Float).toArray)

    override def toString: String =
      List(
        s"timeUSec = $timeUSec",
        s"estimatorType = $estimatorType",
        s"lat = $lat",
        s"lon = $lon",
        s"alt = $alt",
        s"relativeAlt = $relativeAlt",
        s"vX = $vX",
        s"vY = $vY",
        s"vZ = $vZ",
        s"covariance = $covariance"
      ).mkString("GlobalPositionIntCov(", ", ", ")")
  }

  /**
    * 64 LOCAL_POSITION_NED_COV
    *
    * The filtered local position (e.g. fused computer vision and accelerometers). Coordinate frame is right-handed, Z-axis down (aeronautical frame, NED / north-east-down convention)
    *
    * @param timeUSec      Timestamp (microseconds since system boot or since UNIX epoch)
    * @param estimatorType Class id of the estimator this estimate originated from.
    * @param x             X Position
    * @param y             Y Position
    * @param z             Z Position
    * @param vX            X Speed (m/s)
    * @param vY            Y Speed (m/s)
    * @param vZ            Z Speed (m/s)
    * @param aX            X Acceleration (m/s^^2)
    * @param aY            Y Acceleration (m/s^^2)
    * @param aZ            Z Acceleration (m/s^^2)
    * @param covariance    Covariance matrix upper right triangular (first nine entries are the first ROW, next eight entries are the second row, etc.)
    */
  case class LocalPositionNEDCov(
    timeUSec: Long,
    estimatorType: MAVEstimatorType,
    x: Float,
    y: Float,
    z: Float,
    vX: Float,
    vY: Float,
    vZ: Float,
    aX: Float,
    aY: Float,
    aZ: Float,
    covariance: IndexedSeq[Float]
  ) extends MAVMessage("LOCAL_POSITION_NED_COV", systemId, componentId, protocolVersion) {
    set("time_usec", timeUSec)
    set("estimator_type", estimatorType.value)
    set("x", x)
    set("y", y)
    set("z", z)
    set("vx", vX)
    set("vy", vY)
    set("vz", vZ)
    set("ax", aX)
    set("ay", aY)
    set("az", aZ)
    set("covariance", covariance.map(float2Float).toArray)

    override def toString: String =
      List(
        s"timeUSec = $timeUSec",
        s"estimatorType = $estimatorType",
        s"x = $x",
        s"y = $y",
        s"z = $z",
        s"vX = $vX",
        s"vY = $vY",
        s"vZ = $vZ",
        s"aX = $aX",
        s"aY = $aY",
        s"aZ = $aZ",
        s"covariance = $covariance"
      ).mkString("LocalPositionNEDCov(", ", ", ")")
  }

  /**
    * 65 RC_CHANNELS
    *
    * The PPM values of the RC channels received. The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%. Individual receivers/transmitters might violate this specification.
    *
    * @param timeBootMS Timestamp (milliseconds since system boot)
    * @param chanCount  Total number of RC channels being received. This can be larger than 18, indicating that more channels are available but not given in this message. This value should be 0 when no RC channels are available.
    * @param chan1Raw   RC channel 1 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
    * @param chan2Raw   RC channel 2 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
    * @param chan3Raw   RC channel 3 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
    * @param chan4Raw   RC channel 4 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
    * @param chan5Raw   RC channel 5 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
    * @param chan6Raw   RC channel 6 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
    * @param chan7Raw   RC channel 7 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
    * @param chan8Raw   RC channel 8 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
    * @param chan9Raw   RC channel 9 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
    * @param chan10Raw  RC channel 10 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
    * @param chan11Raw  RC channel 11 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
    * @param chan12Raw  RC channel 12 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
    * @param chan13Raw  RC channel 13 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
    * @param chan14Raw  RC channel 14 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
    * @param chan15Raw  RC channel 15 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
    * @param chan16Raw  RC channel 16 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
    * @param chan17Raw  RC channel 17 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
    * @param chan18Raw  RC channel 18 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
    * @param rSSI       Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown.
    */
  case class RCChannels(
    timeBootMS: Int,
    chanCount: Int,
    chan1Raw: Int,
    chan2Raw: Int,
    chan3Raw: Int,
    chan4Raw: Int,
    chan5Raw: Int,
    chan6Raw: Int,
    chan7Raw: Int,
    chan8Raw: Int,
    chan9Raw: Int,
    chan10Raw: Int,
    chan11Raw: Int,
    chan12Raw: Int,
    chan13Raw: Int,
    chan14Raw: Int,
    chan15Raw: Int,
    chan16Raw: Int,
    chan17Raw: Int,
    chan18Raw: Int,
    rSSI: Int
  ) extends MAVMessage("RC_CHANNELS", systemId, componentId, protocolVersion) {
    set("time_boot_ms", timeBootMS)
    set("chancount", chanCount)
    set("chan1_raw", chan1Raw)
    set("chan2_raw", chan2Raw)
    set("chan3_raw", chan3Raw)
    set("chan4_raw", chan4Raw)
    set("chan5_raw", chan5Raw)
    set("chan6_raw", chan6Raw)
    set("chan7_raw", chan7Raw)
    set("chan8_raw", chan8Raw)
    set("chan9_raw", chan9Raw)
    set("chan10_raw", chan10Raw)
    set("chan11_raw", chan11Raw)
    set("chan12_raw", chan12Raw)
    set("chan13_raw", chan13Raw)
    set("chan14_raw", chan14Raw)
    set("chan15_raw", chan15Raw)
    set("chan16_raw", chan16Raw)
    set("chan17_raw", chan17Raw)
    set("chan18_raw", chan18Raw)
    set("rssi", rSSI)

    override def toString: String =
      List(
        s"timeBootMS = $timeBootMS",
        s"chanCount = $chanCount",
        s"chan1Raw = $chan1Raw",
        s"chan2Raw = $chan2Raw",
        s"chan3Raw = $chan3Raw",
        s"chan4Raw = $chan4Raw",
        s"chan5Raw = $chan5Raw",
        s"chan6Raw = $chan6Raw",
        s"chan7Raw = $chan7Raw",
        s"chan8Raw = $chan8Raw",
        s"chan9Raw = $chan9Raw",
        s"chan10Raw = $chan10Raw",
        s"chan11Raw = $chan11Raw",
        s"chan12Raw = $chan12Raw",
        s"chan13Raw = $chan13Raw",
        s"chan14Raw = $chan14Raw",
        s"chan15Raw = $chan15Raw",
        s"chan16Raw = $chan16Raw",
        s"chan17Raw = $chan17Raw",
        s"chan18Raw = $chan18Raw",
        s"rSSI = $rSSI"
      ).mkString("RCChannels(", ", ", ")")
  }

  /**
    * 66 REQUEST_DATA_STREAM
    *
    * THIS INTERFACE IS DEPRECATED. USE SET_MESSAGE_INTERVAL INSTEAD.
    *
    * @param targetSystem    The target requested to send the message stream.
    * @param targetComponent The target requested to send the message stream.
    * @param reqStreamId     The ID of the requested data stream
    * @param reqMessageRate  The requested message rate
    * @param startStop       1 to start sending, 0 to stop sending.
    */
  case class RequestDataStream(
    targetSystem: Int,
    targetComponent: Int,
    reqStreamId: Int,
    reqMessageRate: Int,
    startStop: Int
  ) extends MAVMessage("REQUEST_DATA_STREAM", systemId, componentId, protocolVersion) {
    set("target_system", targetSystem)
    set("target_component", targetComponent)
    set("req_stream_id", reqStreamId)
    set("req_message_rate", reqMessageRate)
    set("start_stop", startStop)

    override def toString: String =
      List(
        s"targetSystem = $targetSystem",
        s"targetComponent = $targetComponent",
        s"reqStreamId = $reqStreamId",
        s"reqMessageRate = $reqMessageRate",
        s"startStop = $startStop"
      ).mkString("RequestDataStream(", ", ", ")")
  }

  /**
    * 67 DATA_STREAM
    *
    * THIS INTERFACE IS DEPRECATED. USE MESSAGE_INTERVAL INSTEAD.
    *
    * @param streamId    The ID of the requested data stream
    * @param messageRate The message rate
    * @param onOff       1 stream is enabled, 0 stream is stopped.
    */
  case class DataStream(
    streamId: Int,
    messageRate: Int,
    onOff: Int
  ) extends MAVMessage("DATA_STREAM", systemId, componentId, protocolVersion) {
    set("stream_id", streamId)
    set("message_rate", messageRate)
    set("on_off", onOff)

    override def toString: String =
      List(
        s"streamId = $streamId",
        s"messageRate = $messageRate",
        s"onOff = $onOff"
      ).mkString("DataStream(", ", ", ")")
  }

  /**
    * 69 MANUAL_CONTROL
    *
    * This message provides an API for manually controlling the vehicle using standard joystick axes nomenclature, along with a joystick-like input device. Unused axes can be disabled an buttons are also transmit as boolean values of their
    *
    * @param target  The system to be controlled.
    * @param x       X-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to forward(1000)-backward(-1000) movement on a joystick and the pitch of a vehicle.
    * @param y       Y-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to left(-1000)-right(1000) movement on a joystick and the roll of a vehicle.
    * @param z       Z-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to a separate slider movement with maximum being 1000 and minimum being -1000 on a joystick and the thrust of a vehicle. Positive values are positive thrust, negative values are negative thrust.
    * @param r       R-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to a twisting of the joystick, with counter-clockwise being 1000 and clockwise being -1000, and the yaw of a vehicle.
    * @param buttons A bitfield corresponding to the joystick buttons' current state, 1 for pressed, 0 for released. The lowest bit corresponds to Button 1.
    */
  case class ManualControl(
    target: Int,
    x: Int,
    y: Int,
    z: Int,
    r: Int,
    buttons: Int
  ) extends MAVMessage("MANUAL_CONTROL", systemId, componentId, protocolVersion) {
    set("target", target)
    set("x", x)
    set("y", y)
    set("z", z)
    set("r", r)
    set("buttons", buttons)

    override def toString: String =
      List(
        s"target = $target",
        s"x = $x",
        s"y = $y",
        s"z = $z",
        s"r = $r",
        s"buttons = $buttons"
      ).mkString("ManualControl(", ", ", ")")
  }

  /**
    * 70 RC_CHANNELS_OVERRIDE
    *
    * The RAW values of the RC channels sent to the MAV to override info received from the RC radio. A value of UINT16_MAX means no change to that channel. A value of 0 means control of that channel should be released back to the RC radio. The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%. Individual receivers/transmitters might violate this specification.
    *
    * @param targetSystem    System ID
    * @param targetComponent Component ID
    * @param chan1Raw        RC channel 1 value, in microseconds. A value of UINT16_MAX means to ignore this field.
    * @param chan2Raw        RC channel 2 value, in microseconds. A value of UINT16_MAX means to ignore this field.
    * @param chan3Raw        RC channel 3 value, in microseconds. A value of UINT16_MAX means to ignore this field.
    * @param chan4Raw        RC channel 4 value, in microseconds. A value of UINT16_MAX means to ignore this field.
    * @param chan5Raw        RC channel 5 value, in microseconds. A value of UINT16_MAX means to ignore this field.
    * @param chan6Raw        RC channel 6 value, in microseconds. A value of UINT16_MAX means to ignore this field.
    * @param chan7Raw        RC channel 7 value, in microseconds. A value of UINT16_MAX means to ignore this field.
    * @param chan8Raw        RC channel 8 value, in microseconds. A value of UINT16_MAX means to ignore this field.
    */
  case class RCChannelsOverride(
    targetSystem: Int,
    targetComponent: Int,
    chan1Raw: Int,
    chan2Raw: Int,
    chan3Raw: Int,
    chan4Raw: Int,
    chan5Raw: Int,
    chan6Raw: Int,
    chan7Raw: Int,
    chan8Raw: Int
  ) extends MAVMessage("RC_CHANNELS_OVERRIDE", systemId, componentId, protocolVersion) {
    set("target_system", targetSystem)
    set("target_component", targetComponent)
    set("chan1_raw", chan1Raw)
    set("chan2_raw", chan2Raw)
    set("chan3_raw", chan3Raw)
    set("chan4_raw", chan4Raw)
    set("chan5_raw", chan5Raw)
    set("chan6_raw", chan6Raw)
    set("chan7_raw", chan7Raw)
    set("chan8_raw", chan8Raw)

    override def toString: String =
      List(
        s"targetSystem = $targetSystem",
        s"targetComponent = $targetComponent",
        s"chan1Raw = $chan1Raw",
        s"chan2Raw = $chan2Raw",
        s"chan3Raw = $chan3Raw",
        s"chan4Raw = $chan4Raw",
        s"chan5Raw = $chan5Raw",
        s"chan6Raw = $chan6Raw",
        s"chan7Raw = $chan7Raw",
        s"chan8Raw = $chan8Raw"
      ).mkString("RCChannelsOverride(", ", ", ")")
  }

  /**
    * 73 MISSION_ITEM_INT
    *
    * Message encoding a mission item. This message is emitted to announce
    * the presence of a mission item and to set a mission item on the system. The mission item can be either in x, y, z meters (type: LOCAL) or x:lat, y:lon, z:altitude. Local frame is Z-down, right handed (NED), global frame is Z-up, right handed (ENU). See alsohttp://qgroundcontrol.org/mavlink/waypoint_protocol.
    *
    * @param targetSystem    System ID
    * @param targetComponent Component ID
    * @param seq             Waypoint ID (sequence number). Starts at zero. Increases monotonically for each waypoint, no gaps in the sequence (0,1,2,3,4).
    * @param frame           The coordinate system of the waypoint. see MAV_FRAME in mavlink_types.h
    * @param command         The scheduled action for the waypoint. see MAV_CMD in common.xml MAVLink specs
    * @param current         false:0, true:1
    * @param autoContinue    autocontinue to next wp
    * @param param1          PARAM1, see MAV_CMD enum
    * @param param2          PARAM2, see MAV_CMD enum
    * @param param3          PARAM3, see MAV_CMD enum
    * @param param4          PARAM4, see MAV_CMD enum
    * @param x               PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^^7
    * @param y               PARAM6 / y position: local: x position in meters * 1e4, global: longitude in degrees *10^^7
    * @param z               PARAM7 / z position: global: altitude in meters (relative or absolute, depending on frame.
    * @param missionType     Mission type, see MAV_MISSION_TYPE
    */
  case class MissionItemInt(
    targetSystem: Int,
    targetComponent: Int,
    seq: Int,
    frame: MAVFrame,
    command: MAVCmd,
    current: Int,
    autoContinue: Int,
    param1: Float,
    param2: Float,
    param3: Float,
    param4: Float,
    x: Int,
    y: Int,
    z: Float,
    missionType: MAVMissionType
  ) extends MAVMessage("MISSION_ITEM_INT", systemId, componentId, protocolVersion) {
    set("target_system", targetSystem)
    set("target_component", targetComponent)
    set("seq", seq)
    set("frame", frame.value)
    set("command", command.value)
    set("current", current)
    set("autocontinue", autoContinue)
    set("param1", param1)
    set("param2", param2)
    set("param3", param3)
    set("param4", param4)
    set("x", x)
    set("y", y)
    set("z", z)
    set("mission_type", missionType.value)

    override def toString: String =
      List(
        s"targetSystem = $targetSystem",
        s"targetComponent = $targetComponent",
        s"seq = $seq",
        s"frame = $frame",
        s"command = $command",
        s"current = $current",
        s"autoContinue = $autoContinue",
        s"param1 = $param1",
        s"param2 = $param2",
        s"param3 = $param3",
        s"param4 = $param4",
        s"x = $x",
        s"y = $y",
        s"z = $z",
        s"missionType = $missionType"
      ).mkString("MissionItemInt(", ", ", ")")
  }

  /**
    * 74 VFR_HUD
    *
    * Metrics typically displayed on a HUD for fixed wing aircraft
    *
    * @param airspeed    Current airspeed in m/s
    * @param groundSpeed Current ground speed in m/s
    * @param heading     Current heading in degrees, in compass units (0..360, 0=north)
    * @param throttle    Current throttle setting in integer percent, 0 to 100
    * @param alt         Current altitude (MSL), in meters
    * @param climb       Current climb rate in meters/second
    */
  case class VFRHUD(
    airspeed: Float,
    groundSpeed: Float,
    heading: Int,
    throttle: Int,
    alt: Float,
    climb: Float
  ) extends MAVMessage("VFR_HUD", systemId, componentId, protocolVersion) {
    set("airspeed", airspeed)
    set("groundspeed", groundSpeed)
    set("heading", heading)
    set("throttle", throttle)
    set("alt", alt)
    set("climb", climb)

    override def toString: String =
      List(
        s"airspeed = $airspeed",
        s"groundSpeed = $groundSpeed",
        s"heading = $heading",
        s"throttle = $throttle",
        s"alt = $alt",
        s"climb = $climb"
      ).mkString("VFRHUD(", ", ", ")")
  }

  /**
    * 75 COMMAND_INT
    *
    * Message encoding a command with parameters as scaled integers. Scaling depends on the actual command value.
    *
    * @param targetSystem    System ID
    * @param targetComponent Component ID
    * @param frame           The coordinate system of the COMMAND. see MAV_FRAME in mavlink_types.h
    * @param command         The scheduled action for the mission item. see MAV_CMD in common.xml MAVLink specs
    * @param current         false:0, true:1
    * @param autoContinue    autocontinue to next wp
    * @param param1          PARAM1, see MAV_CMD enum
    * @param param2          PARAM2, see MAV_CMD enum
    * @param param3          PARAM3, see MAV_CMD enum
    * @param param4          PARAM4, see MAV_CMD enum
    * @param x               PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^^7
    * @param y               PARAM6 / local: y position in meters * 1e4, global: longitude in degrees * 10^^7
    * @param z               PARAM7 / z position: global: altitude in meters (relative or absolute, depending on frame.
    */
  case class CommandInt(
    targetSystem: Int,
    targetComponent: Int,
    frame: MAVFrame,
    command: MAVCmd,
    current: Int,
    autoContinue: Int,
    param1: Float,
    param2: Float,
    param3: Float,
    param4: Float,
    x: Int,
    y: Int,
    z: Float
  ) extends MAVMessage("COMMAND_INT", systemId, componentId, protocolVersion) {
    set("target_system", targetSystem)
    set("target_component", targetComponent)
    set("frame", frame.value)
    set("command", command.value)
    set("current", current)
    set("autocontinue", autoContinue)
    set("param1", param1)
    set("param2", param2)
    set("param3", param3)
    set("param4", param4)
    set("x", x)
    set("y", y)
    set("z", z)

    override def toString: String =
      List(
        s"targetSystem = $targetSystem",
        s"targetComponent = $targetComponent",
        s"frame = $frame",
        s"command = $command",
        s"current = $current",
        s"autoContinue = $autoContinue",
        s"param1 = $param1",
        s"param2 = $param2",
        s"param3 = $param3",
        s"param4 = $param4",
        s"x = $x",
        s"y = $y",
        s"z = $z"
      ).mkString("CommandInt(", ", ", ")")
  }

  /**
    * 76 COMMAND_LONG
    *
    * Send a command with up to seven parameters to the MAV
    *
    * @param targetSystem    System which should execute the command
    * @param targetComponent Component which should execute the command, 0 for all components
    * @param command         Command ID, as defined by MAV_CMD enum.
    * @param confirmation    0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
    * @param param1          Parameter 1, as defined by MAV_CMD enum.
    * @param param2          Parameter 2, as defined by MAV_CMD enum.
    * @param param3          Parameter 3, as defined by MAV_CMD enum.
    * @param param4          Parameter 4, as defined by MAV_CMD enum.
    * @param param5          Parameter 5, as defined by MAV_CMD enum.
    * @param param6          Parameter 6, as defined by MAV_CMD enum.
    * @param param7          Parameter 7, as defined by MAV_CMD enum.
    */
  case class CommandLong(
    targetSystem: Int,
    targetComponent: Int,
    command: MAVCmd,
    confirmation: Int,
    param1: Float,
    param2: Float,
    param3: Float,
    param4: Float,
    param5: Float,
    param6: Float,
    param7: Float
  ) extends MAVMessage("COMMAND_LONG", systemId, componentId, protocolVersion) {
    set("target_system", targetSystem)
    set("target_component", targetComponent)
    set("command", command.value)
    set("confirmation", confirmation)
    set("param1", param1)
    set("param2", param2)
    set("param3", param3)
    set("param4", param4)
    set("param5", param5)
    set("param6", param6)
    set("param7", param7)

    override def toString: String =
      List(
        s"targetSystem = $targetSystem",
        s"targetComponent = $targetComponent",
        s"command = $command",
        s"confirmation = $confirmation",
        s"param1 = $param1",
        s"param2 = $param2",
        s"param3 = $param3",
        s"param4 = $param4",
        s"param5 = $param5",
        s"param6 = $param6",
        s"param7 = $param7"
      ).mkString("CommandLong(", ", ", ")")
  }

  /**
    * 77 COMMAND_ACK
    *
    * Report status of a command. Includes feedback whether the command was executed.
    *
    * @param command         Command ID, as defined by MAV_CMD enum.
    * @param result          See MAV_RESULT enum
    * @param progress        WIP: Also used as result_param1, it can be set with a enum containing the errors reasons of why the command was denied or the progress percentage or 255 if unknown the progress when result is MAV_RESULT_IN_PROGRESS.
    * @param resultParam2    WIP: Additional parameter of the result, example: which parameter of MAV_CMD_NAV_WAYPOINT caused it to be denied.
    * @param targetSystem    WIP: System which requested the command to be executed
    * @param targetComponent WIP: Component which requested the command to be executed
    */
  case class CommandAck(
    command: MAVCmd,
    result: MAVResult,
    progress: Int,
    resultParam2: Int,
    targetSystem: Int,
    targetComponent: Int
  ) extends MAVMessage("COMMAND_ACK", systemId, componentId, protocolVersion) {
    set("command", command.value)
    set("result", result.value)
    set("progress", progress)
    set("result_param2", resultParam2)
    set("target_system", targetSystem)
    set("target_component", targetComponent)

    override def toString: String =
      List(
        s"command = $command",
        s"result = $result",
        s"progress = $progress",
        s"resultParam2 = $resultParam2",
        s"targetSystem = $targetSystem",
        s"targetComponent = $targetComponent"
      ).mkString("CommandAck(", ", ", ")")
  }

  /**
    * 81 MANUAL_SETPOINT
    *
    * Setpoint in roll, pitch, yaw and thrust from the operator
    *
    * @param timeBootMS           Timestamp in milliseconds since system boot
    * @param roll                 Desired roll rate in radians per second
    * @param pitch                Desired pitch rate in radians per second
    * @param yaw                  Desired yaw rate in radians per second
    * @param thrust               Collective thrust, normalized to 0 .. 1
    * @param modeSwitch           Flight mode switch position, 0.. 255
    * @param manualOverrideSwitch Override mode switch position, 0.. 255
    */
  case class ManualSetpoint(
    timeBootMS: Int,
    roll: Float,
    pitch: Float,
    yaw: Float,
    thrust: Float,
    modeSwitch: Int,
    manualOverrideSwitch: Int
  ) extends MAVMessage("MANUAL_SETPOINT", systemId, componentId, protocolVersion) {
    set("time_boot_ms", timeBootMS)
    set("roll", roll)
    set("pitch", pitch)
    set("yaw", yaw)
    set("thrust", thrust)
    set("mode_switch", modeSwitch)
    set("manual_override_switch", manualOverrideSwitch)

    override def toString: String =
      List(
        s"timeBootMS = $timeBootMS",
        s"roll = $roll",
        s"pitch = $pitch",
        s"yaw = $yaw",
        s"thrust = $thrust",
        s"modeSwitch = $modeSwitch",
        s"manualOverrideSwitch = $manualOverrideSwitch"
      ).mkString("ManualSetpoint(", ", ", ")")
  }

  /**
    * 82 SET_ATTITUDE_TARGET
    *
    * Sets a desired vehicle attitude. Used by an external controller to command the vehicle (manual controller or other system).
    *
    * @param timeBootMS      Timestamp in milliseconds since system boot
    * @param targetSystem    System ID
    * @param targetComponent Component ID
    * @param typeMask        Mappings: If any of these bits are set, the corresponding input should be ignored: bit 1: body roll rate, bit 2: body pitch rate, bit 3: body yaw rate. bit 4-bit 6: reserved, bit 7: throttle, bit 8: attitude
    * @param q               Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
    * @param bodyRollRate    Body roll rate in radians per second
    * @param bodyPitchRate   Body roll rate in radians per second
    * @param bodyYawRate     Body roll rate in radians per second
    * @param thrust          Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust)
    */
  case class SetAttitudeTarget(
    timeBootMS: Int,
    targetSystem: Int,
    targetComponent: Int,
    typeMask: Int,
    q: IndexedSeq[Float],
    bodyRollRate: Float,
    bodyPitchRate: Float,
    bodyYawRate: Float,
    thrust: Float
  ) extends MAVMessage("SET_ATTITUDE_TARGET", systemId, componentId, protocolVersion) {
    set("time_boot_ms", timeBootMS)
    set("target_system", targetSystem)
    set("target_component", targetComponent)
    set("type_mask", typeMask)
    set("q", q.map(float2Float).toArray)
    set("body_roll_rate", bodyRollRate)
    set("body_pitch_rate", bodyPitchRate)
    set("body_yaw_rate", bodyYawRate)
    set("thrust", thrust)

    override def toString: String =
      List(
        s"timeBootMS = $timeBootMS",
        s"targetSystem = $targetSystem",
        s"targetComponent = $targetComponent",
        s"typeMask = $typeMask",
        s"q = $q",
        s"bodyRollRate = $bodyRollRate",
        s"bodyPitchRate = $bodyPitchRate",
        s"bodyYawRate = $bodyYawRate",
        s"thrust = $thrust"
      ).mkString("SetAttitudeTarget(", ", ", ")")
  }

  /**
    * 83 ATTITUDE_TARGET
    *
    * Reports the current commanded attitude of the vehicle as specified by the autopilot. This should match the commands sent in a SET_ATTITUDE_TARGET message if the vehicle is being controlled this way.
    *
    * @param timeBootMS    Timestamp in milliseconds since system boot
    * @param typeMask      Mappings: If any of these bits are set, the corresponding input should be ignored: bit 1: body roll rate, bit 2: body pitch rate, bit 3: body yaw rate. bit 4-bit 7: reserved, bit 8: attitude
    * @param q             Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
    * @param bodyRollRate  Body roll rate in radians per second
    * @param bodyPitchRate Body pitch rate in radians per second
    * @param bodyYawRate   Body yaw rate in radians per second
    * @param thrust        Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust)
    */
  case class AttitudeTarget(
    timeBootMS: Int,
    typeMask: Int,
    q: IndexedSeq[Float],
    bodyRollRate: Float,
    bodyPitchRate: Float,
    bodyYawRate: Float,
    thrust: Float
  ) extends MAVMessage("ATTITUDE_TARGET", systemId, componentId, protocolVersion) {
    set("time_boot_ms", timeBootMS)
    set("type_mask", typeMask)
    set("q", q.map(float2Float).toArray)
    set("body_roll_rate", bodyRollRate)
    set("body_pitch_rate", bodyPitchRate)
    set("body_yaw_rate", bodyYawRate)
    set("thrust", thrust)

    override def toString: String =
      List(
        s"timeBootMS = $timeBootMS",
        s"typeMask = $typeMask",
        s"q = $q",
        s"bodyRollRate = $bodyRollRate",
        s"bodyPitchRate = $bodyPitchRate",
        s"bodyYawRate = $bodyYawRate",
        s"thrust = $thrust"
      ).mkString("AttitudeTarget(", ", ", ")")
  }

  /**
    * 84 SET_POSITION_TARGET_LOCAL_NED
    *
    * Sets a desired vehicle position in a local north-east-down coordinate frame. Used by an external controller to command the vehicle (manual controller or other system).
    *
    * @param timeBootMS      Timestamp in milliseconds since system boot
    * @param targetSystem    System ID
    * @param targetComponent Component ID
    * @param coordinateFrame Valid options are: MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7, MAV_FRAME_BODY_NED = 8, MAV_FRAME_BODY_OFFSET_NED = 9
    * @param typeMask        Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint, bit 11: yaw, bit 12: yaw rate
    * @param x               X Position in NED frame in meters
    * @param y               Y Position in NED frame in meters
    * @param z               Z Position in NED frame in meters (note, altitude is negative in NED)
    * @param vX              X velocity in NED frame in meter / s
    * @param vY              Y velocity in NED frame in meter / s
    * @param vZ              Z velocity in NED frame in meter / s
    * @param aFX             X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^^2 or N
    * @param aFY             Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^^2 or N
    * @param aFZ             Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^^2 or N
    * @param yaw             yaw setpoint in rad
    * @param yawRate         yaw rate setpoint in rad/s
    */
  case class SetPositionTargetLocalNED(
    timeBootMS: Int,
    targetSystem: Int,
    targetComponent: Int,
    coordinateFrame: MAVFrame,
    typeMask: Int,
    x: Float,
    y: Float,
    z: Float,
    vX: Float,
    vY: Float,
    vZ: Float,
    aFX: Float,
    aFY: Float,
    aFZ: Float,
    yaw: Float,
    yawRate: Float
  ) extends MAVMessage("SET_POSITION_TARGET_LOCAL_NED", systemId, componentId, protocolVersion) {
    set("time_boot_ms", timeBootMS)
    set("target_system", targetSystem)
    set("target_component", targetComponent)
    set("coordinate_frame", coordinateFrame.value)
    set("type_mask", typeMask)
    set("x", x)
    set("y", y)
    set("z", z)
    set("vx", vX)
    set("vy", vY)
    set("vz", vZ)
    set("afx", aFX)
    set("afy", aFY)
    set("afz", aFZ)
    set("yaw", yaw)
    set("yaw_rate", yawRate)

    override def toString: String =
      List(
        s"timeBootMS = $timeBootMS",
        s"targetSystem = $targetSystem",
        s"targetComponent = $targetComponent",
        s"coordinateFrame = $coordinateFrame",
        s"typeMask = $typeMask",
        s"x = $x",
        s"y = $y",
        s"z = $z",
        s"vX = $vX",
        s"vY = $vY",
        s"vZ = $vZ",
        s"aFX = $aFX",
        s"aFY = $aFY",
        s"aFZ = $aFZ",
        s"yaw = $yaw",
        s"yawRate = $yawRate"
      ).mkString("SetPositionTargetLocalNED(", ", ", ")")
  }

  /**
    * 85 POSITION_TARGET_LOCAL_NED
    *
    * Reports the current commanded vehicle position, velocity, and acceleration as specified by the autopilot. This should match the commands sent in SET_POSITION_TARGET_LOCAL_NED if the vehicle is being controlled this way.
    *
    * @param timeBootMS      Timestamp in milliseconds since system boot
    * @param coordinateFrame Valid options are: MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7, MAV_FRAME_BODY_NED = 8, MAV_FRAME_BODY_OFFSET_NED = 9
    * @param typeMask        Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint, bit 11: yaw, bit 12: yaw rate
    * @param x               X Position in NED frame in meters
    * @param y               Y Position in NED frame in meters
    * @param z               Z Position in NED frame in meters (note, altitude is negative in NED)
    * @param vX              X velocity in NED frame in meter / s
    * @param vY              Y velocity in NED frame in meter / s
    * @param vZ              Z velocity in NED frame in meter / s
    * @param aFX             X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^^2 or N
    * @param aFY             Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^^2 or N
    * @param aFZ             Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^^2 or N
    * @param yaw             yaw setpoint in rad
    * @param yawRate         yaw rate setpoint in rad/s
    */
  case class PositionTargetLocalNED(
    timeBootMS: Int,
    coordinateFrame: MAVFrame,
    typeMask: Int,
    x: Float,
    y: Float,
    z: Float,
    vX: Float,
    vY: Float,
    vZ: Float,
    aFX: Float,
    aFY: Float,
    aFZ: Float,
    yaw: Float,
    yawRate: Float
  ) extends MAVMessage("POSITION_TARGET_LOCAL_NED", systemId, componentId, protocolVersion) {
    set("time_boot_ms", timeBootMS)
    set("coordinate_frame", coordinateFrame.value)
    set("type_mask", typeMask)
    set("x", x)
    set("y", y)
    set("z", z)
    set("vx", vX)
    set("vy", vY)
    set("vz", vZ)
    set("afx", aFX)
    set("afy", aFY)
    set("afz", aFZ)
    set("yaw", yaw)
    set("yaw_rate", yawRate)

    override def toString: String =
      List(
        s"timeBootMS = $timeBootMS",
        s"coordinateFrame = $coordinateFrame",
        s"typeMask = $typeMask",
        s"x = $x",
        s"y = $y",
        s"z = $z",
        s"vX = $vX",
        s"vY = $vY",
        s"vZ = $vZ",
        s"aFX = $aFX",
        s"aFY = $aFY",
        s"aFZ = $aFZ",
        s"yaw = $yaw",
        s"yawRate = $yawRate"
      ).mkString("PositionTargetLocalNED(", ", ", ")")
  }

  /**
    * 86 SET_POSITION_TARGET_GLOBAL_INT
    *
    * Sets a desired vehicle position, velocity, and/or acceleration in a global coordinate system (WGS84). Used by an external controller to command the vehicle (manual controller or other system).
    *
    * @param timeBootMS      Timestamp in milliseconds since system boot. The rationale for the timestamp in the setpoint is to allow the system to compensate for the transport delay of the setpoint. This allows the system to compensate processing latency.
    * @param targetSystem    System ID
    * @param targetComponent Component ID
    * @param coordinateFrame Valid options are: MAV_FRAME_GLOBAL_INT = 5, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6, MAV_FRAME_GLOBAL_TERRAIN_ALT_INT = 11
    * @param typeMask        Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint, bit 11: yaw, bit 12: yaw rate
    * @param latInt          X Position in WGS84 frame in 1e7 * meters
    * @param lonInt          Y Position in WGS84 frame in 1e7 * meters
    * @param alt             Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_INT
    * @param vX              X velocity in NED frame in meter / s
    * @param vY              Y velocity in NED frame in meter / s
    * @param vZ              Z velocity in NED frame in meter / s
    * @param aFX             X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^^2 or N
    * @param aFY             Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^^2 or N
    * @param aFZ             Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^^2 or N
    * @param yaw             yaw setpoint in rad
    * @param yawRate         yaw rate setpoint in rad/s
    */
  case class SetPositionTargetGlobalInt(
    timeBootMS: Int,
    targetSystem: Int,
    targetComponent: Int,
    coordinateFrame: MAVFrame,
    typeMask: Int,
    latInt: Int,
    lonInt: Int,
    alt: Float,
    vX: Float,
    vY: Float,
    vZ: Float,
    aFX: Float,
    aFY: Float,
    aFZ: Float,
    yaw: Float,
    yawRate: Float
  ) extends MAVMessage("SET_POSITION_TARGET_GLOBAL_INT", systemId, componentId, protocolVersion) {
    set("time_boot_ms", timeBootMS)
    set("target_system", targetSystem)
    set("target_component", targetComponent)
    set("coordinate_frame", coordinateFrame.value)
    set("type_mask", typeMask)
    set("lat_int", latInt)
    set("lon_int", lonInt)
    set("alt", alt)
    set("vx", vX)
    set("vy", vY)
    set("vz", vZ)
    set("afx", aFX)
    set("afy", aFY)
    set("afz", aFZ)
    set("yaw", yaw)
    set("yaw_rate", yawRate)

    override def toString: String =
      List(
        s"timeBootMS = $timeBootMS",
        s"targetSystem = $targetSystem",
        s"targetComponent = $targetComponent",
        s"coordinateFrame = $coordinateFrame",
        s"typeMask = $typeMask",
        s"latInt = $latInt",
        s"lonInt = $lonInt",
        s"alt = $alt",
        s"vX = $vX",
        s"vY = $vY",
        s"vZ = $vZ",
        s"aFX = $aFX",
        s"aFY = $aFY",
        s"aFZ = $aFZ",
        s"yaw = $yaw",
        s"yawRate = $yawRate"
      ).mkString("SetPositionTargetGlobalInt(", ", ", ")")
  }

  /**
    * 87 POSITION_TARGET_GLOBAL_INT
    *
    * Reports the current commanded vehicle position, velocity, and acceleration as specified by the autopilot. This should match the commands sent in SET_POSITION_TARGET_GLOBAL_INT if the vehicle is being controlled this way.
    *
    * @param timeBootMS      Timestamp in milliseconds since system boot. The rationale for the timestamp in the setpoint is to allow the system to compensate for the transport delay of the setpoint. This allows the system to compensate processing latency.
    * @param coordinateFrame Valid options are: MAV_FRAME_GLOBAL_INT = 5, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6, MAV_FRAME_GLOBAL_TERRAIN_ALT_INT = 11
    * @param typeMask        Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint, bit 11: yaw, bit 12: yaw rate
    * @param latInt          X Position in WGS84 frame in 1e7 * meters
    * @param lonInt          Y Position in WGS84 frame in 1e7 * meters
    * @param alt             Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_INT
    * @param vX              X velocity in NED frame in meter / s
    * @param vY              Y velocity in NED frame in meter / s
    * @param vZ              Z velocity in NED frame in meter / s
    * @param aFX             X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^^2 or N
    * @param aFY             Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^^2 or N
    * @param aFZ             Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^^2 or N
    * @param yaw             yaw setpoint in rad
    * @param yawRate         yaw rate setpoint in rad/s
    */
  case class PositionTargetGlobalInt(
    timeBootMS: Int,
    coordinateFrame: MAVFrame,
    typeMask: Int,
    latInt: Int,
    lonInt: Int,
    alt: Float,
    vX: Float,
    vY: Float,
    vZ: Float,
    aFX: Float,
    aFY: Float,
    aFZ: Float,
    yaw: Float,
    yawRate: Float
  ) extends MAVMessage("POSITION_TARGET_GLOBAL_INT", systemId, componentId, protocolVersion) {
    set("time_boot_ms", timeBootMS)
    set("coordinate_frame", coordinateFrame.value)
    set("type_mask", typeMask)
    set("lat_int", latInt)
    set("lon_int", lonInt)
    set("alt", alt)
    set("vx", vX)
    set("vy", vY)
    set("vz", vZ)
    set("afx", aFX)
    set("afy", aFY)
    set("afz", aFZ)
    set("yaw", yaw)
    set("yaw_rate", yawRate)

    override def toString: String =
      List(
        s"timeBootMS = $timeBootMS",
        s"coordinateFrame = $coordinateFrame",
        s"typeMask = $typeMask",
        s"latInt = $latInt",
        s"lonInt = $lonInt",
        s"alt = $alt",
        s"vX = $vX",
        s"vY = $vY",
        s"vZ = $vZ",
        s"aFX = $aFX",
        s"aFY = $aFY",
        s"aFZ = $aFZ",
        s"yaw = $yaw",
        s"yawRate = $yawRate"
      ).mkString("PositionTargetGlobalInt(", ", ", ")")
  }

  /**
    * 89 LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET
    *
    * The offset in X, Y, Z and yaw between the LOCAL_POSITION_NED messages of MAV X and the global coordinate frame in NED coordinates. Coordinate frame is right-handed, Z-axis down (aeronautical frame, NED / north-east-down convention)
    *
    * @param timeBootMS Timestamp (milliseconds since system boot)
    * @param x          X Position
    * @param y          Y Position
    * @param z          Z Position
    * @param roll       Roll
    * @param pitch      Pitch
    * @param yaw        Yaw
    */
  case class LocalPositionNEDSystemGlobalOffset(
    timeBootMS: Int,
    x: Float,
    y: Float,
    z: Float,
    roll: Float,
    pitch: Float,
    yaw: Float
  ) extends MAVMessage("LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET", systemId, componentId, protocolVersion) {
    set("time_boot_ms", timeBootMS)
    set("x", x)
    set("y", y)
    set("z", z)
    set("roll", roll)
    set("pitch", pitch)
    set("yaw", yaw)

    override def toString: String =
      List(
        s"timeBootMS = $timeBootMS",
        s"x = $x",
        s"y = $y",
        s"z = $z",
        s"roll = $roll",
        s"pitch = $pitch",
        s"yaw = $yaw"
      ).mkString("LocalPositionNEDSystemGlobalOffset(", ", ", ")")
  }

  /**
    * 90 HIL_STATE
    *
    * DEPRECATED PACKET! Suffers from missing airspeed fields and singularities due to Euler angles. Please use HIL_STATE_QUATERNION instead. Sent from simulation to autopilot. This packet is useful for high throughput applications such as hardware in the loop simulations.
    *
    * @param timeUSec   Timestamp (microseconds since UNIX epoch or microseconds since system boot)
    * @param roll       Roll angle (rad)
    * @param pitch      Pitch angle (rad)
    * @param yaw        Yaw angle (rad)
    * @param rollSpeed  Body frame roll / phi angular speed (rad/s)
    * @param pitchSpeed Body frame pitch / theta angular speed (rad/s)
    * @param yawSpeed   Body frame yaw / psi angular speed (rad/s)
    * @param lat        Latitude, expressed as * 1E7
    * @param lon        Longitude, expressed as * 1E7
    * @param alt        Altitude in meters, expressed as * 1000 (millimeters)
    * @param vX         Ground X Speed (Latitude), expressed as m/s * 100
    * @param vY         Ground Y Speed (Longitude), expressed as m/s * 100
    * @param vZ         Ground Z Speed (Altitude), expressed as m/s * 100
    * @param xAcc       X acceleration (mg)
    * @param yAcc       Y acceleration (mg)
    * @param zAcc       Z acceleration (mg)
    */
  case class HILState(
    timeUSec: Long,
    roll: Float,
    pitch: Float,
    yaw: Float,
    rollSpeed: Float,
    pitchSpeed: Float,
    yawSpeed: Float,
    lat: Int,
    lon: Int,
    alt: Int,
    vX: Int,
    vY: Int,
    vZ: Int,
    xAcc: Int,
    yAcc: Int,
    zAcc: Int
  ) extends MAVMessage("HIL_STATE", systemId, componentId, protocolVersion) {
    set("time_usec", timeUSec)
    set("roll", roll)
    set("pitch", pitch)
    set("yaw", yaw)
    set("rollspeed", rollSpeed)
    set("pitchspeed", pitchSpeed)
    set("yawspeed", yawSpeed)
    set("lat", lat)
    set("lon", lon)
    set("alt", alt)
    set("vx", vX)
    set("vy", vY)
    set("vz", vZ)
    set("xacc", xAcc)
    set("yacc", yAcc)
    set("zacc", zAcc)

    override def toString: String =
      List(
        s"timeUSec = $timeUSec",
        s"roll = $roll",
        s"pitch = $pitch",
        s"yaw = $yaw",
        s"rollSpeed = $rollSpeed",
        s"pitchSpeed = $pitchSpeed",
        s"yawSpeed = $yawSpeed",
        s"lat = $lat",
        s"lon = $lon",
        s"alt = $alt",
        s"vX = $vX",
        s"vY = $vY",
        s"vZ = $vZ",
        s"xAcc = $xAcc",
        s"yAcc = $yAcc",
        s"zAcc = $zAcc"
      ).mkString("HILState(", ", ", ")")
  }

  /**
    * 91 HIL_CONTROLS
    *
    * Sent from autopilot to simulation. Hardware in the loop control outputs
    *
    * @param timeUSec      Timestamp (microseconds since UNIX epoch or microseconds since system boot)
    * @param rollAilerons  Control output -1 .. 1
    * @param pitchElevator Control output -1 .. 1
    * @param yawRudder     Control output -1 .. 1
    * @param throttle      Throttle 0 .. 1
    * @param aux1          Aux 1, -1 .. 1
    * @param aux2          Aux 2, -1 .. 1
    * @param aux3          Aux 3, -1 .. 1
    * @param aux4          Aux 4, -1 .. 1
    * @param mode          System mode (MAV_MODE)
    * @param navMode       Navigation mode (MAV_NAV_MODE)
    */
  case class HILControls(
    timeUSec: Long,
    rollAilerons: Float,
    pitchElevator: Float,
    yawRudder: Float,
    throttle: Float,
    aux1: Float,
    aux2: Float,
    aux3: Float,
    aux4: Float,
    mode: MAVMode,
    navMode: Int
  ) extends MAVMessage("HIL_CONTROLS", systemId, componentId, protocolVersion) {
    set("time_usec", timeUSec)
    set("roll_ailerons", rollAilerons)
    set("pitch_elevator", pitchElevator)
    set("yaw_rudder", yawRudder)
    set("throttle", throttle)
    set("aux1", aux1)
    set("aux2", aux2)
    set("aux3", aux3)
    set("aux4", aux4)
    set("mode", mode.value)
    set("nav_mode", navMode)

    override def toString: String =
      List(
        s"timeUSec = $timeUSec",
        s"rollAilerons = $rollAilerons",
        s"pitchElevator = $pitchElevator",
        s"yawRudder = $yawRudder",
        s"throttle = $throttle",
        s"aux1 = $aux1",
        s"aux2 = $aux2",
        s"aux3 = $aux3",
        s"aux4 = $aux4",
        s"mode = $mode",
        s"navMode = $navMode"
      ).mkString("HILControls(", ", ", ")")
  }

  /**
    * 92 HIL_RC_INPUTS_RAW
    *
    * Sent from simulation to autopilot. The RAW values of the RC channels received. The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%. Individual receivers/transmitters might violate this specification.
    *
    * @param timeUSec  Timestamp (microseconds since UNIX epoch or microseconds since system boot)
    * @param chan1Raw  RC channel 1 value, in microseconds
    * @param chan2Raw  RC channel 2 value, in microseconds
    * @param chan3Raw  RC channel 3 value, in microseconds
    * @param chan4Raw  RC channel 4 value, in microseconds
    * @param chan5Raw  RC channel 5 value, in microseconds
    * @param chan6Raw  RC channel 6 value, in microseconds
    * @param chan7Raw  RC channel 7 value, in microseconds
    * @param chan8Raw  RC channel 8 value, in microseconds
    * @param chan9Raw  RC channel 9 value, in microseconds
    * @param chan10Raw RC channel 10 value, in microseconds
    * @param chan11Raw RC channel 11 value, in microseconds
    * @param chan12Raw RC channel 12 value, in microseconds
    * @param rSSI      Receive signal strength indicator, 0: 0%, 255: 100%
    */
  case class HILRCInputsRaw(
    timeUSec: Long,
    chan1Raw: Int,
    chan2Raw: Int,
    chan3Raw: Int,
    chan4Raw: Int,
    chan5Raw: Int,
    chan6Raw: Int,
    chan7Raw: Int,
    chan8Raw: Int,
    chan9Raw: Int,
    chan10Raw: Int,
    chan11Raw: Int,
    chan12Raw: Int,
    rSSI: Int
  ) extends MAVMessage("HIL_RC_INPUTS_RAW", systemId, componentId, protocolVersion) {
    set("time_usec", timeUSec)
    set("chan1_raw", chan1Raw)
    set("chan2_raw", chan2Raw)
    set("chan3_raw", chan3Raw)
    set("chan4_raw", chan4Raw)
    set("chan5_raw", chan5Raw)
    set("chan6_raw", chan6Raw)
    set("chan7_raw", chan7Raw)
    set("chan8_raw", chan8Raw)
    set("chan9_raw", chan9Raw)
    set("chan10_raw", chan10Raw)
    set("chan11_raw", chan11Raw)
    set("chan12_raw", chan12Raw)
    set("rssi", rSSI)

    override def toString: String =
      List(
        s"timeUSec = $timeUSec",
        s"chan1Raw = $chan1Raw",
        s"chan2Raw = $chan2Raw",
        s"chan3Raw = $chan3Raw",
        s"chan4Raw = $chan4Raw",
        s"chan5Raw = $chan5Raw",
        s"chan6Raw = $chan6Raw",
        s"chan7Raw = $chan7Raw",
        s"chan8Raw = $chan8Raw",
        s"chan9Raw = $chan9Raw",
        s"chan10Raw = $chan10Raw",
        s"chan11Raw = $chan11Raw",
        s"chan12Raw = $chan12Raw",
        s"rSSI = $rSSI"
      ).mkString("HILRCInputsRaw(", ", ", ")")
  }

  /**
    * 93 HIL_ACTUATOR_CONTROLS
    *
    * Sent from autopilot to simulation. Hardware in the loop control outputs (replacement for HIL_CONTROLS)
    *
    * @param timeUSec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
    * @param controls Control outputs -1 .. 1. Channel assignment depends on the simulated hardware.
    * @param mode     System mode (MAV_MODE), includes arming state.
    * @param flags    Flags as bitfield, reserved for future use.
    */
  case class HILActuatorControls(
    timeUSec: Long,
    controls: IndexedSeq[Float],
    mode: MAVMode,
    flags: Long
  ) extends MAVMessage("HIL_ACTUATOR_CONTROLS", systemId, componentId, protocolVersion) {
    set("time_usec", timeUSec)
    set("controls", controls.map(float2Float).toArray)
    set("mode", mode.value)
    set("flags", flags)

    override def toString: String =
      List(
        s"timeUSec = $timeUSec",
        s"controls = $controls",
        s"mode = $mode",
        s"flags = $flags"
      ).mkString("HILActuatorControls(", ", ", ")")
  }

  /**
    * 100 OPTICAL_FLOW
    *
    * Optical flow from a flow sensor (e.g. optical mouse sensor)
    *
    * @param timeUSec       Timestamp (UNIX)
    * @param sensorId       Sensor ID
    * @param flowX          Flow in pixels * 10 in x-sensor direction (dezi-pixels)
    * @param flowY          Flow in pixels * 10 in y-sensor direction (dezi-pixels)
    * @param flowCompMX     Flow in meters in x-sensor direction, angular-speed compensated
    * @param flowCompMY     Flow in meters in y-sensor direction, angular-speed compensated
    * @param quality        Optical flow quality / confidence. 0: bad, 255: maximum quality
    * @param groundDistance Ground distance in meters. Positive value: distance known. Negative value: Unknown distance
    * @param flowRateX      Flow rate in radians/second about X axis
    * @param flowRateY      Flow rate in radians/second about Y axis
    */
  case class OpticalFlow(
    timeUSec: Long,
    sensorId: Int,
    flowX: Int,
    flowY: Int,
    flowCompMX: Float,
    flowCompMY: Float,
    quality: Int,
    groundDistance: Float,
    flowRateX: Float,
    flowRateY: Float
  ) extends MAVMessage("OPTICAL_FLOW", systemId, componentId, protocolVersion) {
    set("time_usec", timeUSec)
    set("sensor_id", sensorId)
    set("flow_x", flowX)
    set("flow_y", flowY)
    set("flow_comp_m_x", flowCompMX)
    set("flow_comp_m_y", flowCompMY)
    set("quality", quality)
    set("ground_distance", groundDistance)
    set("flow_rate_x", flowRateX)
    set("flow_rate_y", flowRateY)

    override def toString: String =
      List(
        s"timeUSec = $timeUSec",
        s"sensorId = $sensorId",
        s"flowX = $flowX",
        s"flowY = $flowY",
        s"flowCompMX = $flowCompMX",
        s"flowCompMY = $flowCompMY",
        s"quality = $quality",
        s"groundDistance = $groundDistance",
        s"flowRateX = $flowRateX",
        s"flowRateY = $flowRateY"
      ).mkString("OpticalFlow(", ", ", ")")
  }

  /**
    * 101 GLOBAL_VISION_POSITION_ESTIMATE
    *
    * @param uSec  Timestamp (microseconds, synced to UNIX time or since system boot)
    * @param x     Global X position
    * @param y     Global Y position
    * @param z     Global Z position
    * @param roll  Roll angle in rad
    * @param pitch Pitch angle in rad
    * @param yaw   Yaw angle in rad
    */
  case class GlobalVisionPositionEstimate(
    uSec: Long,
    x: Float,
    y: Float,
    z: Float,
    roll: Float,
    pitch: Float,
    yaw: Float
  ) extends MAVMessage("GLOBAL_VISION_POSITION_ESTIMATE", systemId, componentId, protocolVersion) {
    set("usec", uSec)
    set("x", x)
    set("y", y)
    set("z", z)
    set("roll", roll)
    set("pitch", pitch)
    set("yaw", yaw)

    override def toString: String =
      List(
        s"uSec = $uSec",
        s"x = $x",
        s"y = $y",
        s"z = $z",
        s"roll = $roll",
        s"pitch = $pitch",
        s"yaw = $yaw"
      ).mkString("GlobalVisionPositionEstimate(", ", ", ")")
  }

  /**
    * 102 VISION_POSITION_ESTIMATE
    *
    * @param uSec  Timestamp (microseconds, synced to UNIX time or since system boot)
    * @param x     Global X position
    * @param y     Global Y position
    * @param z     Global Z position
    * @param roll  Roll angle in rad
    * @param pitch Pitch angle in rad
    * @param yaw   Yaw angle in rad
    */
  case class VisionPositionEstimate(
    uSec: Long,
    x: Float,
    y: Float,
    z: Float,
    roll: Float,
    pitch: Float,
    yaw: Float
  ) extends MAVMessage("VISION_POSITION_ESTIMATE", systemId, componentId, protocolVersion) {
    set("usec", uSec)
    set("x", x)
    set("y", y)
    set("z", z)
    set("roll", roll)
    set("pitch", pitch)
    set("yaw", yaw)

    override def toString: String =
      List(
        s"uSec = $uSec",
        s"x = $x",
        s"y = $y",
        s"z = $z",
        s"roll = $roll",
        s"pitch = $pitch",
        s"yaw = $yaw"
      ).mkString("VisionPositionEstimate(", ", ", ")")
  }

  /**
    * 103 VISION_SPEED_ESTIMATE
    *
    * @param uSec Timestamp (microseconds, synced to UNIX time or since system boot)
    * @param x    Global X speed
    * @param y    Global Y speed
    * @param z    Global Z speed
    */
  case class VisionSpeedEstimate(
    uSec: Long,
    x: Float,
    y: Float,
    z: Float
  ) extends MAVMessage("VISION_SPEED_ESTIMATE", systemId, componentId, protocolVersion) {
    set("usec", uSec)
    set("x", x)
    set("y", y)
    set("z", z)

    override def toString: String =
      List(
        s"uSec = $uSec",
        s"x = $x",
        s"y = $y",
        s"z = $z"
      ).mkString("VisionSpeedEstimate(", ", ", ")")
  }

  /**
    * 104 VICON_POSITION_ESTIMATE
    *
    * @param uSec  Timestamp (microseconds, synced to UNIX time or since system boot)
    * @param x     Global X position
    * @param y     Global Y position
    * @param z     Global Z position
    * @param roll  Roll angle in rad
    * @param pitch Pitch angle in rad
    * @param yaw   Yaw angle in rad
    */
  case class ViconPositionEstimate(
    uSec: Long,
    x: Float,
    y: Float,
    z: Float,
    roll: Float,
    pitch: Float,
    yaw: Float
  ) extends MAVMessage("VICON_POSITION_ESTIMATE", systemId, componentId, protocolVersion) {
    set("usec", uSec)
    set("x", x)
    set("y", y)
    set("z", z)
    set("roll", roll)
    set("pitch", pitch)
    set("yaw", yaw)

    override def toString: String =
      List(
        s"uSec = $uSec",
        s"x = $x",
        s"y = $y",
        s"z = $z",
        s"roll = $roll",
        s"pitch = $pitch",
        s"yaw = $yaw"
      ).mkString("ViconPositionEstimate(", ", ", ")")
  }

  /**
    * 105 HIGHRES_IMU
    *
    * The IMU readings in SI units in NED body frame
    *
    * @param timeUSec      Timestamp (microseconds, synced to UNIX time or since system boot)
    * @param xAcc          X acceleration (m/s^^2)
    * @param yAcc          Y acceleration (m/s^^2)
    * @param zAcc          Z acceleration (m/s^^2)
    * @param xGyro         Angular speed around X axis (rad / sec)
    * @param yGyro         Angular speed around Y axis (rad / sec)
    * @param zGyro         Angular speed around Z axis (rad / sec)
    * @param xMag          X Magnetic field (Gauss)
    * @param yMag          Y Magnetic field (Gauss)
    * @param zMag          Z Magnetic field (Gauss)
    * @param absPressure   Absolute pressure in millibar
    * @param diffPressure  Differential pressure in millibar
    * @param pressureAlt   Altitude calculated from pressure
    * @param temperature   Temperature in degrees celsius
    * @param fieldsUpdated Bitmask for fields that have updated since last message, bit 0 = xacc, bit 12: temperature
    */
  case class HighresIMU(
    timeUSec: Long,
    xAcc: Float,
    yAcc: Float,
    zAcc: Float,
    xGyro: Float,
    yGyro: Float,
    zGyro: Float,
    xMag: Float,
    yMag: Float,
    zMag: Float,
    absPressure: Float,
    diffPressure: Float,
    pressureAlt: Float,
    temperature: Float,
    fieldsUpdated: Int
  ) extends MAVMessage("HIGHRES_IMU", systemId, componentId, protocolVersion) {
    set("time_usec", timeUSec)
    set("xacc", xAcc)
    set("yacc", yAcc)
    set("zacc", zAcc)
    set("xgyro", xGyro)
    set("ygyro", yGyro)
    set("zgyro", zGyro)
    set("xmag", xMag)
    set("ymag", yMag)
    set("zmag", zMag)
    set("abs_pressure", absPressure)
    set("diff_pressure", diffPressure)
    set("pressure_alt", pressureAlt)
    set("temperature", temperature)
    set("fields_updated", fieldsUpdated)

    override def toString: String =
      List(
        s"timeUSec = $timeUSec",
        s"xAcc = $xAcc",
        s"yAcc = $yAcc",
        s"zAcc = $zAcc",
        s"xGyro = $xGyro",
        s"yGyro = $yGyro",
        s"zGyro = $zGyro",
        s"xMag = $xMag",
        s"yMag = $yMag",
        s"zMag = $zMag",
        s"absPressure = $absPressure",
        s"diffPressure = $diffPressure",
        s"pressureAlt = $pressureAlt",
        s"temperature = $temperature",
        s"fieldsUpdated = $fieldsUpdated"
      ).mkString("HighresIMU(", ", ", ")")
  }

  /**
    * 106 OPTICAL_FLOW_RAD
    *
    * Optical flow from an angular rate flow sensor (e.g. PX4FLOW or mouse sensor)
    *
    * @param timeUSec            Timestamp (microseconds, synced to UNIX time or since system boot)
    * @param sensorId            Sensor ID
    * @param integrationTimeUS   Integration time in microseconds. Divide integrated_x and integrated_y by the integration time to obtain average flow. The integration time also indicates the.
    * @param integratedX         Flow in radians around X axis (Sensor RH rotation about the X axis induces a positive flow. Sensor linear motion along the positive Y axis induces a negative flow.)
    * @param integratedY         Flow in radians around Y axis (Sensor RH rotation about the Y axis induces a positive flow. Sensor linear motion along the positive X axis induces a positive flow.)
    * @param integratedXGyro     RH rotation around X axis (rad)
    * @param integratedYGyro     RH rotation around Y axis (rad)
    * @param integratedZGyro     RH rotation around Z axis (rad)
    * @param temperature         Temperature * 100 in centi-degrees Celsius
    * @param quality             Optical flow quality / confidence. 0: no valid flow, 255: maximum quality
    * @param timeDeltaDistanceUS Time in microseconds since the distance was sampled.
    * @param distance            Distance to the center of the flow field in meters. Positive value (including zero): distance known. Negative value: Unknown distance.
    */
  case class OpticalFlowRad(
    timeUSec: Long,
    sensorId: Int,
    integrationTimeUS: Int,
    integratedX: Float,
    integratedY: Float,
    integratedXGyro: Float,
    integratedYGyro: Float,
    integratedZGyro: Float,
    temperature: Int,
    quality: Int,
    timeDeltaDistanceUS: Int,
    distance: Float
  ) extends MAVMessage("OPTICAL_FLOW_RAD", systemId, componentId, protocolVersion) {
    set("time_usec", timeUSec)
    set("sensor_id", sensorId)
    set("integration_time_us", integrationTimeUS)
    set("integrated_x", integratedX)
    set("integrated_y", integratedY)
    set("integrated_xgyro", integratedXGyro)
    set("integrated_ygyro", integratedYGyro)
    set("integrated_zgyro", integratedZGyro)
    set("temperature", temperature)
    set("quality", quality)
    set("time_delta_distance_us", timeDeltaDistanceUS)
    set("distance", distance)

    override def toString: String =
      List(
        s"timeUSec = $timeUSec",
        s"sensorId = $sensorId",
        s"integrationTimeUS = $integrationTimeUS",
        s"integratedX = $integratedX",
        s"integratedY = $integratedY",
        s"integratedXGyro = $integratedXGyro",
        s"integratedYGyro = $integratedYGyro",
        s"integratedZGyro = $integratedZGyro",
        s"temperature = $temperature",
        s"quality = $quality",
        s"timeDeltaDistanceUS = $timeDeltaDistanceUS",
        s"distance = $distance"
      ).mkString("OpticalFlowRad(", ", ", ")")
  }

  /**
    * 107 HIL_SENSOR
    *
    * The IMU readings in SI units in NED body frame
    *
    * @param timeUSec      Timestamp (microseconds, synced to UNIX time or since system boot)
    * @param xAcc          X acceleration (m/s^^2)
    * @param yAcc          Y acceleration (m/s^^2)
    * @param zAcc          Z acceleration (m/s^^2)
    * @param xGyro         Angular speed around X axis in body frame (rad / sec)
    * @param yGyro         Angular speed around Y axis in body frame (rad / sec)
    * @param zGyro         Angular speed around Z axis in body frame (rad / sec)
    * @param xMag          X Magnetic field (Gauss)
    * @param yMag          Y Magnetic field (Gauss)
    * @param zMag          Z Magnetic field (Gauss)
    * @param absPressure   Absolute pressure in millibar
    * @param diffPressure  Differential pressure (airspeed) in millibar
    * @param pressureAlt   Altitude calculated from pressure
    * @param temperature   Temperature in degrees celsius
    * @param fieldsUpdated Bitmask for fields that have updated since last message, bit 0 = xacc, bit 12: temperature, bit 31: full reset of attitude/position/velocities/etc was performed in sim.
    */
  case class HILSensor(
    timeUSec: Long,
    xAcc: Float,
    yAcc: Float,
    zAcc: Float,
    xGyro: Float,
    yGyro: Float,
    zGyro: Float,
    xMag: Float,
    yMag: Float,
    zMag: Float,
    absPressure: Float,
    diffPressure: Float,
    pressureAlt: Float,
    temperature: Float,
    fieldsUpdated: Int
  ) extends MAVMessage("HIL_SENSOR", systemId, componentId, protocolVersion) {
    set("time_usec", timeUSec)
    set("xacc", xAcc)
    set("yacc", yAcc)
    set("zacc", zAcc)
    set("xgyro", xGyro)
    set("ygyro", yGyro)
    set("zgyro", zGyro)
    set("xmag", xMag)
    set("ymag", yMag)
    set("zmag", zMag)
    set("abs_pressure", absPressure)
    set("diff_pressure", diffPressure)
    set("pressure_alt", pressureAlt)
    set("temperature", temperature)
    set("fields_updated", fieldsUpdated)

    override def toString: String =
      List(
        s"timeUSec = $timeUSec",
        s"xAcc = $xAcc",
        s"yAcc = $yAcc",
        s"zAcc = $zAcc",
        s"xGyro = $xGyro",
        s"yGyro = $yGyro",
        s"zGyro = $zGyro",
        s"xMag = $xMag",
        s"yMag = $yMag",
        s"zMag = $zMag",
        s"absPressure = $absPressure",
        s"diffPressure = $diffPressure",
        s"pressureAlt = $pressureAlt",
        s"temperature = $temperature",
        s"fieldsUpdated = $fieldsUpdated"
      ).mkString("HILSensor(", ", ", ")")
  }

  /**
    * 108 SIM_STATE
    *
    * Status of simulation environment, if used
    *
    * @param q1         True attitude quaternion component 1, w (1 in null-rotation)
    * @param q2         True attitude quaternion component 2, x (0 in null-rotation)
    * @param q3         True attitude quaternion component 3, y (0 in null-rotation)
    * @param q4         True attitude quaternion component 4, z (0 in null-rotation)
    * @param roll       Attitude roll expressed as Euler angles, not recommended except for human-readable outputs
    * @param pitch      Attitude pitch expressed as Euler angles, not recommended except for human-readable outputs
    * @param yaw        Attitude yaw expressed as Euler angles, not recommended except for human-readable outputs
    * @param xAcc       X acceleration m/s/s
    * @param yAcc       Y acceleration m/s/s
    * @param zAcc       Z acceleration m/s/s
    * @param xGyro      Angular speed around X axis rad/s
    * @param yGyro      Angular speed around Y axis rad/s
    * @param zGyro      Angular speed around Z axis rad/s
    * @param lat        Latitude in degrees
    * @param lon        Longitude in degrees
    * @param alt        Altitude in meters
    * @param stdDevHorz Horizontal position standard deviation
    * @param stdDevVert Vertical position standard deviation
    * @param vN         True velocity in m/s in NORTH direction in earth-fixed NED frame
    * @param vE         True velocity in m/s in EAST direction in earth-fixed NED frame
    * @param vD         True velocity in m/s in DOWN direction in earth-fixed NED frame
    */
  case class SimState(
    q1: Float,
    q2: Float,
    q3: Float,
    q4: Float,
    roll: Float,
    pitch: Float,
    yaw: Float,
    xAcc: Float,
    yAcc: Float,
    zAcc: Float,
    xGyro: Float,
    yGyro: Float,
    zGyro: Float,
    lat: Float,
    lon: Float,
    alt: Float,
    stdDevHorz: Float,
    stdDevVert: Float,
    vN: Float,
    vE: Float,
    vD: Float
  ) extends MAVMessage("SIM_STATE", systemId, componentId, protocolVersion) {
    set("q1", q1)
    set("q2", q2)
    set("q3", q3)
    set("q4", q4)
    set("roll", roll)
    set("pitch", pitch)
    set("yaw", yaw)
    set("xacc", xAcc)
    set("yacc", yAcc)
    set("zacc", zAcc)
    set("xgyro", xGyro)
    set("ygyro", yGyro)
    set("zgyro", zGyro)
    set("lat", lat)
    set("lon", lon)
    set("alt", alt)
    set("std_dev_horz", stdDevHorz)
    set("std_dev_vert", stdDevVert)
    set("vn", vN)
    set("ve", vE)
    set("vd", vD)

    override def toString: String =
      List(
        s"q1 = $q1",
        s"q2 = $q2",
        s"q3 = $q3",
        s"q4 = $q4",
        s"roll = $roll",
        s"pitch = $pitch",
        s"yaw = $yaw",
        s"xAcc = $xAcc",
        s"yAcc = $yAcc",
        s"zAcc = $zAcc",
        s"xGyro = $xGyro",
        s"yGyro = $yGyro",
        s"zGyro = $zGyro",
        s"lat = $lat",
        s"lon = $lon",
        s"alt = $alt",
        s"stdDevHorz = $stdDevHorz",
        s"stdDevVert = $stdDevVert",
        s"vN = $vN",
        s"vE = $vE",
        s"vD = $vD"
      ).mkString("SimState(", ", ", ")")
  }

  /**
    * 109 RADIO_STATUS
    *
    * Status generated by radio and injected into MAVLink stream.
    *
    * @param rSSI     Local signal strength
    * @param remRSSI  Remote signal strength
    * @param tXBuf    Remaining free buffer space in percent.
    * @param noise    Background noise level
    * @param remNoise Remote background noise level
    * @param rXErrors Receive errors
    * @param fixed    Count of error corrected packets
    */
  case class RadioStatus(
    rSSI: Int,
    remRSSI: Int,
    tXBuf: Int,
    noise: Int,
    remNoise: Int,
    rXErrors: Int,
    fixed: Int
  ) extends MAVMessage("RADIO_STATUS", systemId, componentId, protocolVersion) {
    set("rssi", rSSI)
    set("remrssi", remRSSI)
    set("txbuf", tXBuf)
    set("noise", noise)
    set("remnoise", remNoise)
    set("rxerrors", rXErrors)
    set("fixed", fixed)

    override def toString: String =
      List(
        s"rSSI = $rSSI",
        s"remRSSI = $remRSSI",
        s"tXBuf = $tXBuf",
        s"noise = $noise",
        s"remNoise = $remNoise",
        s"rXErrors = $rXErrors",
        s"fixed = $fixed"
      ).mkString("RadioStatus(", ", ", ")")
  }

  /**
    * 110 FILE_TRANSFER_PROTOCOL
    *
    * File transfer message
    *
    * @param targetNetwork   Network ID (0 for broadcast)
    * @param targetSystem    System ID (0 for broadcast)
    * @param targetComponent Component ID (0 for broadcast)
    * @param payload         Variable length payload. The length is defined by the remaining message length when subtracting the header and other fields.  The entire content of this block is opaque unless you understand any the encoding message_type.  The particular encoding used can be extension specific and might not always be documented as part of the mavlink specification.
    */
  case class FileTransferProtocol(
    targetNetwork: Int,
    targetSystem: Int,
    targetComponent: Int,
    payload: IndexedSeq[Int]
  ) extends MAVMessage("FILE_TRANSFER_PROTOCOL", systemId, componentId, protocolVersion) {
    set("target_network", targetNetwork)
    set("target_system", targetSystem)
    set("target_component", targetComponent)
    set("payload", payload.map(int2Integer).toArray)

    override def toString: String =
      List(
        s"targetNetwork = $targetNetwork",
        s"targetSystem = $targetSystem",
        s"targetComponent = $targetComponent",
        s"payload = $payload"
      ).mkString("FileTransferProtocol(", ", ", ")")
  }

  /**
    * 111 TIMESYNC
    *
    * Time synchronization message.
    *
    * @param tc1 Time sync timestamp 1
    * @param ts1 Time sync timestamp 2
    */
  case class Timesync(
    tc1: Long,
    ts1: Long
  ) extends MAVMessage("TIMESYNC", systemId, componentId, protocolVersion) {
    set("tc1", tc1)
    set("ts1", ts1)

    override def toString: String =
      List(
        s"tc1 = $tc1",
        s"ts1 = $ts1"
      ).mkString("Timesync(", ", ", ")")
  }

  /**
    * 112 CAMERA_TRIGGER
    *
    * Camera-IMU triggering and synchronisation message.
    *
    * @param timeUSec Timestamp for the image frame in microseconds
    * @param seq      Image frame sequence
    */
  case class CameraTrigger(
    timeUSec: Long,
    seq: Int
  ) extends MAVMessage("CAMERA_TRIGGER", systemId, componentId, protocolVersion) {
    set("time_usec", timeUSec)
    set("seq", seq)

    override def toString: String =
      List(
        s"timeUSec = $timeUSec",
        s"seq = $seq"
      ).mkString("CameraTrigger(", ", ", ")")
  }

  /**
    * 113 HIL_GPS
    *
    * The global position, as returned by the Global Positioning System (GPS). This is
    * NOT the global position estimate of the sytem, but rather a RAW sensor value. See message GLOBAL_POSITION for the global position estimate. Coordinate frame is right-handed, Z-axis up (GPS frame).
    *
    * @param timeUSec          Timestamp (microseconds since UNIX epoch or microseconds since system boot)
    * @param fixType           0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
    * @param lat               Latitude (WGS84), in degrees * 1E7
    * @param lon               Longitude (WGS84), in degrees * 1E7
    * @param alt               Altitude (AMSL, not WGS84), in meters * 1000 (positive for up)
    * @param epH               GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
    * @param epV               GPS VDOP vertical dilution of position in cm (m*100). If unknown, set to: 65535
    * @param vel               GPS ground speed in cm/s. If unknown, set to: 65535
    * @param vN                GPS velocity in cm/s in NORTH direction in earth-fixed NED frame
    * @param vE                GPS velocity in cm/s in EAST direction in earth-fixed NED frame
    * @param vD                GPS velocity in cm/s in DOWN direction in earth-fixed NED frame
    * @param cOG               Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: 65535
    * @param satellitesVisible Number of satellites visible. If unknown, set to 255
    */
  case class HILGPS(
    timeUSec: Long,
    fixType: Int,
    lat: Int,
    lon: Int,
    alt: Int,
    epH: Int,
    epV: Int,
    vel: Int,
    vN: Int,
    vE: Int,
    vD: Int,
    cOG: Int,
    satellitesVisible: Int
  ) extends MAVMessage("HIL_GPS", systemId, componentId, protocolVersion) {
    set("time_usec", timeUSec)
    set("fix_type", fixType)
    set("lat", lat)
    set("lon", lon)
    set("alt", alt)
    set("eph", epH)
    set("epv", epV)
    set("vel", vel)
    set("vn", vN)
    set("ve", vE)
    set("vd", vD)
    set("cog", cOG)
    set("satellites_visible", satellitesVisible)

    override def toString: String =
      List(
        s"timeUSec = $timeUSec",
        s"fixType = $fixType",
        s"lat = $lat",
        s"lon = $lon",
        s"alt = $alt",
        s"epH = $epH",
        s"epV = $epV",
        s"vel = $vel",
        s"vN = $vN",
        s"vE = $vE",
        s"vD = $vD",
        s"cOG = $cOG",
        s"satellitesVisible = $satellitesVisible"
      ).mkString("HILGPS(", ", ", ")")
  }

  /**
    * 114 HIL_OPTICAL_FLOW
    *
    * Simulated optical flow from a flow sensor (e.g. PX4FLOW or optical mouse sensor)
    *
    * @param timeUSec            Timestamp (microseconds, synced to UNIX time or since system boot)
    * @param sensorId            Sensor ID
    * @param integrationTimeUS   Integration time in microseconds. Divide integrated_x and integrated_y by the integration time to obtain average flow. The integration time also indicates the.
    * @param integratedX         Flow in radians around X axis (Sensor RH rotation about the X axis induces a positive flow. Sensor linear motion along the positive Y axis induces a negative flow.)
    * @param integratedY         Flow in radians around Y axis (Sensor RH rotation about the Y axis induces a positive flow. Sensor linear motion along the positive X axis induces a positive flow.)
    * @param integratedXGyro     RH rotation around X axis (rad)
    * @param integratedYGyro     RH rotation around Y axis (rad)
    * @param integratedZGyro     RH rotation around Z axis (rad)
    * @param temperature         Temperature * 100 in centi-degrees Celsius
    * @param quality             Optical flow quality / confidence. 0: no valid flow, 255: maximum quality
    * @param timeDeltaDistanceUS Time in microseconds since the distance was sampled.
    * @param distance            Distance to the center of the flow field in meters. Positive value (including zero): distance known. Negative value: Unknown distance.
    */
  case class HILOpticalFlow(
    timeUSec: Long,
    sensorId: Int,
    integrationTimeUS: Int,
    integratedX: Float,
    integratedY: Float,
    integratedXGyro: Float,
    integratedYGyro: Float,
    integratedZGyro: Float,
    temperature: Int,
    quality: Int,
    timeDeltaDistanceUS: Int,
    distance: Float
  ) extends MAVMessage("HIL_OPTICAL_FLOW", systemId, componentId, protocolVersion) {
    set("time_usec", timeUSec)
    set("sensor_id", sensorId)
    set("integration_time_us", integrationTimeUS)
    set("integrated_x", integratedX)
    set("integrated_y", integratedY)
    set("integrated_xgyro", integratedXGyro)
    set("integrated_ygyro", integratedYGyro)
    set("integrated_zgyro", integratedZGyro)
    set("temperature", temperature)
    set("quality", quality)
    set("time_delta_distance_us", timeDeltaDistanceUS)
    set("distance", distance)

    override def toString: String =
      List(
        s"timeUSec = $timeUSec",
        s"sensorId = $sensorId",
        s"integrationTimeUS = $integrationTimeUS",
        s"integratedX = $integratedX",
        s"integratedY = $integratedY",
        s"integratedXGyro = $integratedXGyro",
        s"integratedYGyro = $integratedYGyro",
        s"integratedZGyro = $integratedZGyro",
        s"temperature = $temperature",
        s"quality = $quality",
        s"timeDeltaDistanceUS = $timeDeltaDistanceUS",
        s"distance = $distance"
      ).mkString("HILOpticalFlow(", ", ", ")")
  }

  /**
    * 115 HIL_STATE_QUATERNION
    *
    * Sent from simulation to autopilot, avoids in contrast to HIL_STATE singularities. This packet is useful for high throughput applications such as hardware in the loop simulations.
    *
    * @param timeUSec           Timestamp (microseconds since UNIX epoch or microseconds since system boot)
    * @param attitudeQuaternion Vehicle attitude expressed as normalized quaternion in w, x, y, z order (with 1 0 0 0 being the null-rotation)
    * @param rollSpeed          Body frame roll / phi angular speed (rad/s)
    * @param pitchSpeed         Body frame pitch / theta angular speed (rad/s)
    * @param yawSpeed           Body frame yaw / psi angular speed (rad/s)
    * @param lat                Latitude, expressed as * 1E7
    * @param lon                Longitude, expressed as * 1E7
    * @param alt                Altitude in meters, expressed as * 1000 (millimeters)
    * @param vX                 Ground X Speed (Latitude), expressed as cm/s
    * @param vY                 Ground Y Speed (Longitude), expressed as cm/s
    * @param vZ                 Ground Z Speed (Altitude), expressed as cm/s
    * @param indAirspeed        Indicated airspeed, expressed as cm/s
    * @param trueAirspeed       True airspeed, expressed as cm/s
    * @param xAcc               X acceleration (mg)
    * @param yAcc               Y acceleration (mg)
    * @param zAcc               Z acceleration (mg)
    */
  case class HILStateQuaternion(
    timeUSec: Long,
    attitudeQuaternion: IndexedSeq[Float],
    rollSpeed: Float,
    pitchSpeed: Float,
    yawSpeed: Float,
    lat: Int,
    lon: Int,
    alt: Int,
    vX: Int,
    vY: Int,
    vZ: Int,
    indAirspeed: Int,
    trueAirspeed: Int,
    xAcc: Int,
    yAcc: Int,
    zAcc: Int
  ) extends MAVMessage("HIL_STATE_QUATERNION", systemId, componentId, protocolVersion) {
    set("time_usec", timeUSec)
    set("attitude_quaternion", attitudeQuaternion.map(float2Float).toArray)
    set("rollspeed", rollSpeed)
    set("pitchspeed", pitchSpeed)
    set("yawspeed", yawSpeed)
    set("lat", lat)
    set("lon", lon)
    set("alt", alt)
    set("vx", vX)
    set("vy", vY)
    set("vz", vZ)
    set("ind_airspeed", indAirspeed)
    set("true_airspeed", trueAirspeed)
    set("xacc", xAcc)
    set("yacc", yAcc)
    set("zacc", zAcc)

    override def toString: String =
      List(
        s"timeUSec = $timeUSec",
        s"attitudeQuaternion = $attitudeQuaternion",
        s"rollSpeed = $rollSpeed",
        s"pitchSpeed = $pitchSpeed",
        s"yawSpeed = $yawSpeed",
        s"lat = $lat",
        s"lon = $lon",
        s"alt = $alt",
        s"vX = $vX",
        s"vY = $vY",
        s"vZ = $vZ",
        s"indAirspeed = $indAirspeed",
        s"trueAirspeed = $trueAirspeed",
        s"xAcc = $xAcc",
        s"yAcc = $yAcc",
        s"zAcc = $zAcc"
      ).mkString("HILStateQuaternion(", ", ", ")")
  }

  /**
    * 116 SCALED_IMU2
    *
    * The RAW IMU readings for secondary 9DOF sensor setup. This message should contain the scaled values to the described units
    *
    * @param timeBootMS Timestamp (milliseconds since system boot)
    * @param xAcc       X acceleration (mg)
    * @param yAcc       Y acceleration (mg)
    * @param zAcc       Z acceleration (mg)
    * @param xGyro      Angular speed around X axis (millirad /sec)
    * @param yGyro      Angular speed around Y axis (millirad /sec)
    * @param zGyro      Angular speed around Z axis (millirad /sec)
    * @param xMag       X Magnetic field (milli tesla)
    * @param yMag       Y Magnetic field (milli tesla)
    * @param zMag       Z Magnetic field (milli tesla)
    */
  case class ScaledIMU2(
    timeBootMS: Int,
    xAcc: Int,
    yAcc: Int,
    zAcc: Int,
    xGyro: Int,
    yGyro: Int,
    zGyro: Int,
    xMag: Int,
    yMag: Int,
    zMag: Int
  ) extends MAVMessage("SCALED_IMU2", systemId, componentId, protocolVersion) {
    set("time_boot_ms", timeBootMS)
    set("xacc", xAcc)
    set("yacc", yAcc)
    set("zacc", zAcc)
    set("xgyro", xGyro)
    set("ygyro", yGyro)
    set("zgyro", zGyro)
    set("xmag", xMag)
    set("ymag", yMag)
    set("zmag", zMag)

    override def toString: String =
      List(
        s"timeBootMS = $timeBootMS",
        s"xAcc = $xAcc",
        s"yAcc = $yAcc",
        s"zAcc = $zAcc",
        s"xGyro = $xGyro",
        s"yGyro = $yGyro",
        s"zGyro = $zGyro",
        s"xMag = $xMag",
        s"yMag = $yMag",
        s"zMag = $zMag"
      ).mkString("ScaledIMU2(", ", ", ")")
  }

  /**
    * 117 LOG_REQUEST_LIST
    *
    * Request a list of available logs. On some systems calling this may stop on-board logging until LOG_REQUEST_END is called.
    *
    * @param targetSystem    System ID
    * @param targetComponent Component ID
    * @param start           First log id (0 for first available)
    * @param end             Last log id (0xffff for last available)
    */
  case class LogRequestList(
    targetSystem: Int,
    targetComponent: Int,
    start: Int,
    end: Int
  ) extends MAVMessage("LOG_REQUEST_LIST", systemId, componentId, protocolVersion) {
    set("target_system", targetSystem)
    set("target_component", targetComponent)
    set("start", start)
    set("end", end)

    override def toString: String =
      List(
        s"targetSystem = $targetSystem",
        s"targetComponent = $targetComponent",
        s"start = $start",
        s"end = $end"
      ).mkString("LogRequestList(", ", ", ")")
  }

  /**
    * 118 LOG_ENTRY
    *
    * Reply to LOG_REQUEST_LIST
    *
    * @param id         Log id
    * @param numLogs    Total number of logs
    * @param lastLogNum High log number
    * @param timeUTC    UTC timestamp of log in seconds since 1970, or 0 if not available
    * @param size       Size of the log (may be approximate) in bytes
    */
  case class LogEntry(
    id: Int,
    numLogs: Int,
    lastLogNum: Int,
    timeUTC: Int,
    size: Int
  ) extends MAVMessage("LOG_ENTRY", systemId, componentId, protocolVersion) {
    set("id", id)
    set("num_logs", numLogs)
    set("last_log_num", lastLogNum)
    set("time_utc", timeUTC)
    set("size", size)

    override def toString: String =
      List(
        s"id = $id",
        s"numLogs = $numLogs",
        s"lastLogNum = $lastLogNum",
        s"timeUTC = $timeUTC",
        s"size = $size"
      ).mkString("LogEntry(", ", ", ")")
  }

  /**
    * 119 LOG_REQUEST_DATA
    *
    * Request a chunk of a log
    *
    * @param targetSystem    System ID
    * @param targetComponent Component ID
    * @param id              Log id (from LOG_ENTRY reply)
    * @param ofs             Offset into the log
    * @param count           Number of bytes
    */
  case class LogRequestData(
    targetSystem: Int,
    targetComponent: Int,
    id: Int,
    ofs: Int,
    count: Int
  ) extends MAVMessage("LOG_REQUEST_DATA", systemId, componentId, protocolVersion) {
    set("target_system", targetSystem)
    set("target_component", targetComponent)
    set("id", id)
    set("ofs", ofs)
    set("count", count)

    override def toString: String =
      List(
        s"targetSystem = $targetSystem",
        s"targetComponent = $targetComponent",
        s"id = $id",
        s"ofs = $ofs",
        s"count = $count"
      ).mkString("LogRequestData(", ", ", ")")
  }

  /**
    * 120 LOG_DATA
    *
    * Reply to LOG_REQUEST_DATA
    *
    * @param id    Log id (from LOG_ENTRY reply)
    * @param ofs   Offset into the log
    * @param count Number of bytes (zero for end of log)
    * @param data  log data
    */
  case class LogData(
    id: Int,
    ofs: Int,
    count: Int,
    data: IndexedSeq[Int]
  ) extends MAVMessage("LOG_DATA", systemId, componentId, protocolVersion) {
    set("id", id)
    set("ofs", ofs)
    set("count", count)
    set("data", data.map(int2Integer).toArray)

    override def toString: String =
      List(
        s"id = $id",
        s"ofs = $ofs",
        s"count = $count",
        s"data = $data"
      ).mkString("LogData(", ", ", ")")
  }

  /**
    * 121 LOG_ERASE
    *
    * Erase all logs
    *
    * @param targetSystem    System ID
    * @param targetComponent Component ID
    */
  case class LogErase(
    targetSystem: Int,
    targetComponent: Int
  ) extends MAVMessage("LOG_ERASE", systemId, componentId, protocolVersion) {
    set("target_system", targetSystem)
    set("target_component", targetComponent)

    override def toString: String =
      List(
        s"targetSystem = $targetSystem",
        s"targetComponent = $targetComponent"
      ).mkString("LogErase(", ", ", ")")
  }

  /**
    * 122 LOG_REQUEST_END
    *
    * Stop log transfer and resume normal logging
    *
    * @param targetSystem    System ID
    * @param targetComponent Component ID
    */
  case class LogRequestEnd(
    targetSystem: Int,
    targetComponent: Int
  ) extends MAVMessage("LOG_REQUEST_END", systemId, componentId, protocolVersion) {
    set("target_system", targetSystem)
    set("target_component", targetComponent)

    override def toString: String =
      List(
        s"targetSystem = $targetSystem",
        s"targetComponent = $targetComponent"
      ).mkString("LogRequestEnd(", ", ", ")")
  }

  /**
    * 123 GPS_INJECT_DATA
    *
    * data for injecting into the onboard GPS (used for DGPS)
    *
    * @param targetSystem    System ID
    * @param targetComponent Component ID
    * @param len             data length
    * @param data            raw data (110 is enough for 12 satellites of RTCMv2)
    */
  case class GPSInjectData(
    targetSystem: Int,
    targetComponent: Int,
    len: Int,
    data: IndexedSeq[Int]
  ) extends MAVMessage("GPS_INJECT_DATA", systemId, componentId, protocolVersion) {
    set("target_system", targetSystem)
    set("target_component", targetComponent)
    set("len", len)
    set("data", data.map(int2Integer).toArray)

    override def toString: String =
      List(
        s"targetSystem = $targetSystem",
        s"targetComponent = $targetComponent",
        s"len = $len",
        s"data = $data"
      ).mkString("GPSInjectData(", ", ", ")")
  }

  /**
    * 124 GPS2_RAW
    *
    * Second GPS data. Coordinate frame is right-handed, Z-axis up (GPS frame).
    *
    * @param timeUSec          Timestamp (microseconds since UNIX epoch or microseconds since system boot)
    * @param fixType           See the GPS_FIX_TYPE enum.
    * @param lat               Latitude (WGS84), in degrees * 1E7
    * @param lon               Longitude (WGS84), in degrees * 1E7
    * @param alt               Altitude (AMSL, not WGS84), in meters * 1000 (positive for up)
    * @param epH               GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: UINT16_MAX
    * @param epV               GPS VDOP vertical dilution of position in cm (m*100). If unknown, set to: UINT16_MAX
    * @param vel               GPS ground speed (m/s * 100). If unknown, set to: UINT16_MAX
    * @param cOG               Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
    * @param satellitesVisible Number of satellites visible. If unknown, set to 255
    * @param dGPSNumCh         Number of DGPS satellites
    * @param dGPSAge           Age of DGPS info
    */
  case class GPS2Raw(
    timeUSec: Long,
    fixType: GPSFixType,
    lat: Int,
    lon: Int,
    alt: Int,
    epH: Int,
    epV: Int,
    vel: Int,
    cOG: Int,
    satellitesVisible: Int,
    dGPSNumCh: Int,
    dGPSAge: Int
  ) extends MAVMessage("GPS2_RAW", systemId, componentId, protocolVersion) {
    set("time_usec", timeUSec)
    set("fix_type", fixType.value)
    set("lat", lat)
    set("lon", lon)
    set("alt", alt)
    set("eph", epH)
    set("epv", epV)
    set("vel", vel)
    set("cog", cOG)
    set("satellites_visible", satellitesVisible)
    set("dgps_numch", dGPSNumCh)
    set("dgps_age", dGPSAge)

    override def toString: String =
      List(
        s"timeUSec = $timeUSec",
        s"fixType = $fixType",
        s"lat = $lat",
        s"lon = $lon",
        s"alt = $alt",
        s"epH = $epH",
        s"epV = $epV",
        s"vel = $vel",
        s"cOG = $cOG",
        s"satellitesVisible = $satellitesVisible",
        s"dGPSNumCh = $dGPSNumCh",
        s"dGPSAge = $dGPSAge"
      ).mkString("GPS2Raw(", ", ", ")")
  }

  /**
    * 125 POWER_STATUS
    *
    * Power supply status
    *
    * @param vcc    5V rail voltage in millivolts
    * @param vServo servo rail voltage in millivolts
    * @param flags  power supply status flags (see MAV_POWER_STATUS enum)
    */
  case class PowerStatus(
    vcc: Int,
    vServo: Int,
    flags: Set[MAVPowerStatus]
  ) extends MAVMessage("POWER_STATUS", systemId, componentId, protocolVersion) {
    set("Vcc", vcc)
    set("Vservo", vServo)
    set("flags", flags.foldLeft(0)(_ | _.value))

    override def toString: String =
      List(
        s"vcc = $vcc",
        s"vServo = $vServo",
        s"flags = $flags"
      ).mkString("PowerStatus(", ", ", ")")
  }

  /**
    * 126 SERIAL_CONTROL
    *
    * Control a serial port. This can be used for raw access to an onboard serial peripheral such as a GPS or telemetry radio. It is designed to make it possible to update the devices firmware via MAVLink messages or change the devices settings. A message with zero bytes can be used to change just the baudrate.
    *
    * @param device   See SERIAL_CONTROL_DEV enum
    * @param flags    See SERIAL_CONTROL_FLAG enum
    * @param timeout  Timeout for reply data in milliseconds
    * @param baudRate Baudrate of transfer. Zero means no change.
    * @param count    how many bytes in this transfer
    * @param data     serial data
    */
  case class SerialControl(
    device: SerialControlDev,
    flags: Set[SerialControlFlag],
    timeout: Int,
    baudRate: Int,
    count: Int,
    data: IndexedSeq[Int]
  ) extends MAVMessage("SERIAL_CONTROL", systemId, componentId, protocolVersion) {
    set("device", device.value)
    set("flags", flags.foldLeft(0)(_ | _.value))
    set("timeout", timeout)
    set("baudrate", baudRate)
    set("count", count)
    set("data", data.map(int2Integer).toArray)

    override def toString: String =
      List(
        s"device = $device",
        s"flags = $flags",
        s"timeout = $timeout",
        s"baudRate = $baudRate",
        s"count = $count",
        s"data = $data"
      ).mkString("SerialControl(", ", ", ")")
  }

  /**
    * 127 GPS_RTK
    *
    * RTK GPS data. Gives information on the relative baseline calculation the GPS is reporting
    *
    * @param timeLastBaselineMS Time since boot of last baseline message received in ms.
    * @param rTKReceiverId      Identification of connected RTK receiver.
    * @param wN                 GPS Week Number of last baseline
    * @param tOW                GPS Time of Week of last baseline
    * @param rTKHealth          GPS-specific health report for RTK data.
    * @param rTKRate            Rate of baseline messages being received by GPS, in HZ
    * @param nSats              Current number of sats used for RTK calculation.
    * @param baselineCoordsType Coordinate system of baseline. 0 == ECEF, 1 == NED
    * @param baselineAMM        Current baseline in ECEF x or NED north component in mm.
    * @param baselineBMM        Current baseline in ECEF y or NED east component in mm.
    * @param baselineCMM        Current baseline in ECEF z or NED down component in mm.
    * @param accuracy           Current estimate of baseline accuracy.
    * @param iARNumHypotheses   Current number of integer ambiguity hypotheses.
    */
  case class GPSRTK(
    timeLastBaselineMS: Int,
    rTKReceiverId: Int,
    wN: Int,
    tOW: Int,
    rTKHealth: Int,
    rTKRate: Int,
    nSats: Int,
    baselineCoordsType: Int,
    baselineAMM: Int,
    baselineBMM: Int,
    baselineCMM: Int,
    accuracy: Int,
    iARNumHypotheses: Int
  ) extends MAVMessage("GPS_RTK", systemId, componentId, protocolVersion) {
    set("time_last_baseline_ms", timeLastBaselineMS)
    set("rtk_receiver_id", rTKReceiverId)
    set("wn", wN)
    set("tow", tOW)
    set("rtk_health", rTKHealth)
    set("rtk_rate", rTKRate)
    set("nsats", nSats)
    set("baseline_coords_type", baselineCoordsType)
    set("baseline_a_mm", baselineAMM)
    set("baseline_b_mm", baselineBMM)
    set("baseline_c_mm", baselineCMM)
    set("accuracy", accuracy)
    set("iar_num_hypotheses", iARNumHypotheses)

    override def toString: String =
      List(
        s"timeLastBaselineMS = $timeLastBaselineMS",
        s"rTKReceiverId = $rTKReceiverId",
        s"wN = $wN",
        s"tOW = $tOW",
        s"rTKHealth = $rTKHealth",
        s"rTKRate = $rTKRate",
        s"nSats = $nSats",
        s"baselineCoordsType = $baselineCoordsType",
        s"baselineAMM = $baselineAMM",
        s"baselineBMM = $baselineBMM",
        s"baselineCMM = $baselineCMM",
        s"accuracy = $accuracy",
        s"iARNumHypotheses = $iARNumHypotheses"
      ).mkString("GPSRTK(", ", ", ")")
  }

  /**
    * 128 GPS2_RTK
    *
    * RTK GPS data. Gives information on the relative baseline calculation the GPS is reporting
    *
    * @param timeLastBaselineMS Time since boot of last baseline message received in ms.
    * @param rTKReceiverId      Identification of connected RTK receiver.
    * @param wN                 GPS Week Number of last baseline
    * @param tOW                GPS Time of Week of last baseline
    * @param rTKHealth          GPS-specific health report for RTK data.
    * @param rTKRate            Rate of baseline messages being received by GPS, in HZ
    * @param nSats              Current number of sats used for RTK calculation.
    * @param baselineCoordsType Coordinate system of baseline. 0 == ECEF, 1 == NED
    * @param baselineAMM        Current baseline in ECEF x or NED north component in mm.
    * @param baselineBMM        Current baseline in ECEF y or NED east component in mm.
    * @param baselineCMM        Current baseline in ECEF z or NED down component in mm.
    * @param accuracy           Current estimate of baseline accuracy.
    * @param iARNumHypotheses   Current number of integer ambiguity hypotheses.
    */
  case class GPS2RTK(
    timeLastBaselineMS: Int,
    rTKReceiverId: Int,
    wN: Int,
    tOW: Int,
    rTKHealth: Int,
    rTKRate: Int,
    nSats: Int,
    baselineCoordsType: Int,
    baselineAMM: Int,
    baselineBMM: Int,
    baselineCMM: Int,
    accuracy: Int,
    iARNumHypotheses: Int
  ) extends MAVMessage("GPS2_RTK", systemId, componentId, protocolVersion) {
    set("time_last_baseline_ms", timeLastBaselineMS)
    set("rtk_receiver_id", rTKReceiverId)
    set("wn", wN)
    set("tow", tOW)
    set("rtk_health", rTKHealth)
    set("rtk_rate", rTKRate)
    set("nsats", nSats)
    set("baseline_coords_type", baselineCoordsType)
    set("baseline_a_mm", baselineAMM)
    set("baseline_b_mm", baselineBMM)
    set("baseline_c_mm", baselineCMM)
    set("accuracy", accuracy)
    set("iar_num_hypotheses", iARNumHypotheses)

    override def toString: String =
      List(
        s"timeLastBaselineMS = $timeLastBaselineMS",
        s"rTKReceiverId = $rTKReceiverId",
        s"wN = $wN",
        s"tOW = $tOW",
        s"rTKHealth = $rTKHealth",
        s"rTKRate = $rTKRate",
        s"nSats = $nSats",
        s"baselineCoordsType = $baselineCoordsType",
        s"baselineAMM = $baselineAMM",
        s"baselineBMM = $baselineBMM",
        s"baselineCMM = $baselineCMM",
        s"accuracy = $accuracy",
        s"iARNumHypotheses = $iARNumHypotheses"
      ).mkString("GPS2RTK(", ", ", ")")
  }

  /**
    * 129 SCALED_IMU3
    *
    * The RAW IMU readings for 3rd 9DOF sensor setup. This message should contain the scaled values to the described units
    *
    * @param timeBootMS Timestamp (milliseconds since system boot)
    * @param xAcc       X acceleration (mg)
    * @param yAcc       Y acceleration (mg)
    * @param zAcc       Z acceleration (mg)
    * @param xGyro      Angular speed around X axis (millirad /sec)
    * @param yGyro      Angular speed around Y axis (millirad /sec)
    * @param zGyro      Angular speed around Z axis (millirad /sec)
    * @param xMag       X Magnetic field (milli tesla)
    * @param yMag       Y Magnetic field (milli tesla)
    * @param zMag       Z Magnetic field (milli tesla)
    */
  case class ScaledIMU3(
    timeBootMS: Int,
    xAcc: Int,
    yAcc: Int,
    zAcc: Int,
    xGyro: Int,
    yGyro: Int,
    zGyro: Int,
    xMag: Int,
    yMag: Int,
    zMag: Int
  ) extends MAVMessage("SCALED_IMU3", systemId, componentId, protocolVersion) {
    set("time_boot_ms", timeBootMS)
    set("xacc", xAcc)
    set("yacc", yAcc)
    set("zacc", zAcc)
    set("xgyro", xGyro)
    set("ygyro", yGyro)
    set("zgyro", zGyro)
    set("xmag", xMag)
    set("ymag", yMag)
    set("zmag", zMag)

    override def toString: String =
      List(
        s"timeBootMS = $timeBootMS",
        s"xAcc = $xAcc",
        s"yAcc = $yAcc",
        s"zAcc = $zAcc",
        s"xGyro = $xGyro",
        s"yGyro = $yGyro",
        s"zGyro = $zGyro",
        s"xMag = $xMag",
        s"yMag = $yMag",
        s"zMag = $zMag"
      ).mkString("ScaledIMU3(", ", ", ")")
  }

  /**
    * 130 DATA_TRANSMISSION_HANDSHAKE
    *
    * @param `type`     type of requested/acknowledged data (as defined in ENUM DATA_TYPES in mavlink/include/mavlink_types.h)
    * @param size       total data size in bytes (set on ACK only)
    * @param width      Width of a matrix or image
    * @param height     Height of a matrix or image
    * @param packets    number of packets beeing sent (set on ACK only)
    * @param payload    payload size per packet (normally 253 byte, see DATA field size in message ENCAPSULATED_DATA) (set on ACK only)
    * @param jPGQuality JPEG quality out of [1,100]
    */
  case class DataTransmissionHandshake(
    `type`: Int,
    size: Int,
    width: Int,
    height: Int,
    packets: Int,
    payload: Int,
    jPGQuality: Int
  ) extends MAVMessage("DATA_TRANSMISSION_HANDSHAKE", systemId, componentId, protocolVersion) {
    set("type", `type`)
    set("size", size)
    set("width", width)
    set("height", height)
    set("packets", packets)
    set("payload", payload)
    set("jpg_quality", jPGQuality)

    override def toString: String =
      List(
        s"type = ${`type`}",
        s"size = $size",
        s"width = $width",
        s"height = $height",
        s"packets = $packets",
        s"payload = $payload",
        s"jPGQuality = $jPGQuality"
      ).mkString("DataTransmissionHandshake(", ", ", ")")
  }

  /**
    * 131 ENCAPSULATED_DATA
    *
    * @param seqNr sequence number (starting with 0 on every transmission)
    * @param data  image data bytes
    */
  case class EncapsulatedData(
    seqNr: Int,
    data: IndexedSeq[Int]
  ) extends MAVMessage("ENCAPSULATED_DATA", systemId, componentId, protocolVersion) {
    set("seqnr", seqNr)
    set("data", data.map(int2Integer).toArray)

    override def toString: String =
      List(
        s"seqNr = $seqNr",
        s"data = $data"
      ).mkString("EncapsulatedData(", ", ", ")")
  }

  /**
    * 132 DISTANCE_SENSOR
    *
    * @param timeBootMS      Time since system boot
    * @param minDistance     Minimum distance the sensor can measure in centimeters
    * @param maxDistance     Maximum distance the sensor can measure in centimeters
    * @param currentDistance Current distance reading
    * @param `type`          Type from MAV_DISTANCE_SENSOR enum.
    * @param id              Onboard ID of the sensor
    * @param orientation     Direction the sensor faces from MAV_SENSOR_ORIENTATION enum. downward-facing: ROTATION_PITCH_270, upward-facing: ROTATION_PITCH_90, backward-facing: ROTATION_PITCH_180, forward-facing: ROTATION_NONE, left-facing: ROTATION_YAW_90, right-facing: ROTATION_YAW_270
    * @param covariance      Measurement covariance in centimeters, 0 for unknown / invalid readings
    */
  case class DistanceSensor(
    timeBootMS: Int,
    minDistance: Int,
    maxDistance: Int,
    currentDistance: Int,
    `type`: MAVDistanceSensor,
    id: Int,
    orientation: MAVSensorOrientation,
    covariance: Int
  ) extends MAVMessage("DISTANCE_SENSOR", systemId, componentId, protocolVersion) {
    set("time_boot_ms", timeBootMS)
    set("min_distance", minDistance)
    set("max_distance", maxDistance)
    set("current_distance", currentDistance)
    set("type", `type`.value)
    set("id", id)
    set("orientation", orientation.value)
    set("covariance", covariance)

    override def toString: String =
      List(
        s"timeBootMS = $timeBootMS",
        s"minDistance = $minDistance",
        s"maxDistance = $maxDistance",
        s"currentDistance = $currentDistance",
        s"type = ${`type`}",
        s"id = $id",
        s"orientation = $orientation",
        s"covariance = $covariance"
      ).mkString("DistanceSensor(", ", ", ")")
  }

  /**
    * 133 TERRAIN_REQUEST
    *
    * Request for terrain data and terrain status
    *
    * @param lat         Latitude of SW corner of first grid (degrees *10^^7)
    * @param lon         Longitude of SW corner of first grid (in degrees *10^^7)
    * @param gridSpacing Grid spacing in meters
    * @param mask        Bitmask of requested 4x4 grids (row major 8x7 array of grids, 56 bits)
    */
  case class TerrainRequest(
    lat: Int,
    lon: Int,
    gridSpacing: Int,
    mask: Long
  ) extends MAVMessage("TERRAIN_REQUEST", systemId, componentId, protocolVersion) {
    set("lat", lat)
    set("lon", lon)
    set("grid_spacing", gridSpacing)
    set("mask", mask)

    override def toString: String =
      List(
        s"lat = $lat",
        s"lon = $lon",
        s"gridSpacing = $gridSpacing",
        s"mask = $mask"
      ).mkString("TerrainRequest(", ", ", ")")
  }

  /**
    * 134 TERRAIN_DATA
    *
    * Terrain data sent from GCS. The lat/lon and grid_spacing must be the same as a lat/lon from a TERRAIN_REQUEST
    *
    * @param lat         Latitude of SW corner of first grid (degrees *10^^7)
    * @param lon         Longitude of SW corner of first grid (in degrees *10^^7)
    * @param gridSpacing Grid spacing in meters
    * @param gridBit     bit within the terrain request mask
    * @param data        Terrain data in meters AMSL
    */
  case class TerrainData(
    lat: Int,
    lon: Int,
    gridSpacing: Int,
    gridBit: Int,
    data: IndexedSeq[Int]
  ) extends MAVMessage("TERRAIN_DATA", systemId, componentId, protocolVersion) {
    set("lat", lat)
    set("lon", lon)
    set("grid_spacing", gridSpacing)
    set("gridbit", gridBit)
    set("data", data.map(int2Integer).toArray)

    override def toString: String =
      List(
        s"lat = $lat",
        s"lon = $lon",
        s"gridSpacing = $gridSpacing",
        s"gridBit = $gridBit",
        s"data = $data"
      ).mkString("TerrainData(", ", ", ")")
  }

  /**
    * 135 TERRAIN_CHECK
    *
    * Request that the vehicle report terrain height at the given location. Used by GCS to check if vehicle has all terrain data needed for a mission.
    *
    * @param lat Latitude (degrees *10^^7)
    * @param lon Longitude (degrees *10^^7)
    */
  case class TerrainCheck(
    lat: Int,
    lon: Int
  ) extends MAVMessage("TERRAIN_CHECK", systemId, componentId, protocolVersion) {
    set("lat", lat)
    set("lon", lon)

    override def toString: String =
      List(
        s"lat = $lat",
        s"lon = $lon"
      ).mkString("TerrainCheck(", ", ", ")")
  }

  /**
    * 136 TERRAIN_REPORT
    *
    * Response from a TERRAIN_CHECK request
    *
    * @param lat           Latitude (degrees *10^^7)
    * @param lon           Longitude (degrees *10^^7)
    * @param spacing       grid spacing (zero if terrain at this location unavailable)
    * @param terrainHeight Terrain height in meters AMSL
    * @param currentHeight Current vehicle height above lat/lon terrain height (meters)
    * @param pending       Number of 4x4 terrain blocks waiting to be received or read from disk
    * @param loaded        Number of 4x4 terrain blocks in memory
    */
  case class TerrainReport(
    lat: Int,
    lon: Int,
    spacing: Int,
    terrainHeight: Float,
    currentHeight: Float,
    pending: Int,
    loaded: Int
  ) extends MAVMessage("TERRAIN_REPORT", systemId, componentId, protocolVersion) {
    set("lat", lat)
    set("lon", lon)
    set("spacing", spacing)
    set("terrain_height", terrainHeight)
    set("current_height", currentHeight)
    set("pending", pending)
    set("loaded", loaded)

    override def toString: String =
      List(
        s"lat = $lat",
        s"lon = $lon",
        s"spacing = $spacing",
        s"terrainHeight = $terrainHeight",
        s"currentHeight = $currentHeight",
        s"pending = $pending",
        s"loaded = $loaded"
      ).mkString("TerrainReport(", ", ", ")")
  }

  /**
    * 137 SCALED_PRESSURE2
    *
    * Barometer readings for 2nd barometer
    *
    * @param timeBootMS  Timestamp (milliseconds since system boot)
    * @param pressAbs    Absolute pressure (hectopascal)
    * @param pressDiff   Differential pressure 1 (hectopascal)
    * @param temperature Temperature measurement (0.01 degrees celsius)
    */
  case class ScaledPressure2(
    timeBootMS: Int,
    pressAbs: Float,
    pressDiff: Float,
    temperature: Int
  ) extends MAVMessage("SCALED_PRESSURE2", systemId, componentId, protocolVersion) {
    set("time_boot_ms", timeBootMS)
    set("press_abs", pressAbs)
    set("press_diff", pressDiff)
    set("temperature", temperature)

    override def toString: String =
      List(
        s"timeBootMS = $timeBootMS",
        s"pressAbs = $pressAbs",
        s"pressDiff = $pressDiff",
        s"temperature = $temperature"
      ).mkString("ScaledPressure2(", ", ", ")")
  }

  /**
    * 138 ATT_POS_MOCAP
    *
    * Motion capture attitude and position
    *
    * @param timeUSec Timestamp (micros since boot or Unix epoch)
    * @param q        Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
    * @param x        X position in meters (NED)
    * @param y        Y position in meters (NED)
    * @param z        Z position in meters (NED)
    */
  case class AttPosMocap(
    timeUSec: Long,
    q: IndexedSeq[Float],
    x: Float,
    y: Float,
    z: Float
  ) extends MAVMessage("ATT_POS_MOCAP", systemId, componentId, protocolVersion) {
    set("time_usec", timeUSec)
    set("q", q.map(float2Float).toArray)
    set("x", x)
    set("y", y)
    set("z", z)

    override def toString: String =
      List(
        s"timeUSec = $timeUSec",
        s"q = $q",
        s"x = $x",
        s"y = $y",
        s"z = $z"
      ).mkString("AttPosMocap(", ", ", ")")
  }

  /**
    * 139 SET_ACTUATOR_CONTROL_TARGET
    *
    * Set the vehicle attitude and body angular rates.
    *
    * @param timeUSec        Timestamp (micros since boot or Unix epoch)
    * @param groupMLX        Actuator group. The "_mlx" indicates this is a multi-instance message and a MAVLink parser should use this field to difference between instances.
    * @param targetSystem    System ID
    * @param targetComponent Component ID
    * @param controls        Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0): (index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through mixer to repurpose them as generic outputs.
    */
  case class SetActuatorControlTarget(
    timeUSec: Long,
    groupMLX: Int,
    targetSystem: Int,
    targetComponent: Int,
    controls: IndexedSeq[Float]
  ) extends MAVMessage("SET_ACTUATOR_CONTROL_TARGET", systemId, componentId, protocolVersion) {
    set("time_usec", timeUSec)
    set("group_mlx", groupMLX)
    set("target_system", targetSystem)
    set("target_component", targetComponent)
    set("controls", controls.map(float2Float).toArray)

    override def toString: String =
      List(
        s"timeUSec = $timeUSec",
        s"groupMLX = $groupMLX",
        s"targetSystem = $targetSystem",
        s"targetComponent = $targetComponent",
        s"controls = $controls"
      ).mkString("SetActuatorControlTarget(", ", ", ")")
  }

  /**
    * 140 ACTUATOR_CONTROL_TARGET
    *
    * Set the vehicle attitude and body angular rates.
    *
    * @param timeUSec Timestamp (micros since boot or Unix epoch)
    * @param groupMLX Actuator group. The "_mlx" indicates this is a multi-instance message and a MAVLink parser should use this field to difference between instances.
    * @param controls Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0): (index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through mixer to repurpose them as generic outputs.
    */
  case class ActuatorControlTarget(
    timeUSec: Long,
    groupMLX: Int,
    controls: IndexedSeq[Float]
  ) extends MAVMessage("ACTUATOR_CONTROL_TARGET", systemId, componentId, protocolVersion) {
    set("time_usec", timeUSec)
    set("group_mlx", groupMLX)
    set("controls", controls.map(float2Float).toArray)

    override def toString: String =
      List(
        s"timeUSec = $timeUSec",
        s"groupMLX = $groupMLX",
        s"controls = $controls"
      ).mkString("ActuatorControlTarget(", ", ", ")")
  }

  /**
    * 141 ALTITUDE
    *
    * The current system altitude.
    *
    * @param timeUSec          Timestamp (micros since boot or Unix epoch)
    * @param altitudeMonotonic This altitude measure is initialized on system boot and monotonic (it is never reset, but represents the local altitude change). The only guarantee on this field is that it will never be reset and is consistent within a flight. The recommended value for this field is the uncorrected barometric altitude at boot time. This altitude will also drift and vary between flights.
    * @param altitudeAMSL      This altitude measure is strictly above mean sea level and might be non-monotonic (it might reset on events like GPS lock or when a new QNH value is set). It should be the altitude to which global altitude waypoints are compared to. Note that it is *not* the GPS altitude, however, most GPS modules already output AMSL by default and not the WGS84 altitude.
    * @param altitudeLocal     This is the local altitude in the local coordinate frame. It is not the altitude above home, but in reference to the coordinate origin (0, 0, 0). It is up-positive.
    * @param altitudeRelative  This is the altitude above the home position. It resets on each change of the current home position.
    * @param altitudeTerrain   This is the altitude above terrain. It might be fed by a terrain database or an altimeter. Values smaller than -1000 should be interpreted as unknown.
    * @param bottomClearance   This is not the altitude, but the clear space below the system according to the fused clearance estimate. It generally should max out at the maximum range of e.g. the laser altimeter. It is generally a moving target. A negative value indicates no measurement available.
    */
  case class Altitude(
    timeUSec: Long,
    altitudeMonotonic: Float,
    altitudeAMSL: Float,
    altitudeLocal: Float,
    altitudeRelative: Float,
    altitudeTerrain: Float,
    bottomClearance: Float
  ) extends MAVMessage("ALTITUDE", systemId, componentId, protocolVersion) {
    set("time_usec", timeUSec)
    set("altitude_monotonic", altitudeMonotonic)
    set("altitude_amsl", altitudeAMSL)
    set("altitude_local", altitudeLocal)
    set("altitude_relative", altitudeRelative)
    set("altitude_terrain", altitudeTerrain)
    set("bottom_clearance", bottomClearance)

    override def toString: String =
      List(
        s"timeUSec = $timeUSec",
        s"altitudeMonotonic = $altitudeMonotonic",
        s"altitudeAMSL = $altitudeAMSL",
        s"altitudeLocal = $altitudeLocal",
        s"altitudeRelative = $altitudeRelative",
        s"altitudeTerrain = $altitudeTerrain",
        s"bottomClearance = $bottomClearance"
      ).mkString("Altitude(", ", ", ")")
  }

  /**
    * 142 RESOURCE_REQUEST
    *
    * The autopilot is requesting a resource (file, binary, other type of data)
    *
    * @param requestId    Request ID. This ID should be re-used when sending back URI contents
    * @param uRIType      The type of requested URI. 0 = a file via URL. 1 = a UAVCAN binary
    * @param uRI          The requested unique resource identifier (URI). It is not necessarily a straight domain name (depends on the URI type enum)
    * @param transferType The way the autopilot wants to receive the URI. 0 = MAVLink FTP. 1 = binary stream.
    * @param storage      The storage path the autopilot wants the URI to be stored in. Will only be valid if the transfer_type has a storage associated (e.g. MAVLink FTP).
    */
  case class ResourceRequest(
    requestId: Int,
    uRIType: Int,
    uRI: IndexedSeq[Int],
    transferType: Int,
    storage: IndexedSeq[Int]
  ) extends MAVMessage("RESOURCE_REQUEST", systemId, componentId, protocolVersion) {
    set("request_id", requestId)
    set("uri_type", uRIType)
    set("uri", uRI.map(int2Integer).toArray)
    set("transfer_type", transferType)
    set("storage", storage.map(int2Integer).toArray)

    override def toString: String =
      List(
        s"requestId = $requestId",
        s"uRIType = $uRIType",
        s"uRI = $uRI",
        s"transferType = $transferType",
        s"storage = $storage"
      ).mkString("ResourceRequest(", ", ", ")")
  }

  /**
    * 143 SCALED_PRESSURE3
    *
    * Barometer readings for 3rd barometer
    *
    * @param timeBootMS  Timestamp (milliseconds since system boot)
    * @param pressAbs    Absolute pressure (hectopascal)
    * @param pressDiff   Differential pressure 1 (hectopascal)
    * @param temperature Temperature measurement (0.01 degrees celsius)
    */
  case class ScaledPressure3(
    timeBootMS: Int,
    pressAbs: Float,
    pressDiff: Float,
    temperature: Int
  ) extends MAVMessage("SCALED_PRESSURE3", systemId, componentId, protocolVersion) {
    set("time_boot_ms", timeBootMS)
    set("press_abs", pressAbs)
    set("press_diff", pressDiff)
    set("temperature", temperature)

    override def toString: String =
      List(
        s"timeBootMS = $timeBootMS",
        s"pressAbs = $pressAbs",
        s"pressDiff = $pressDiff",
        s"temperature = $temperature"
      ).mkString("ScaledPressure3(", ", ", ")")
  }

  /**
    * 144 FOLLOW_TARGET
    *
    * current motion information from a designated system
    *
    * @param timestamp       Timestamp in milliseconds since system boot
    * @param estCapabilities bit positions for tracker reporting capabilities (POS = 0, VEL = 1, ACCEL = 2, ATT + RATES = 3)
    * @param lat             Latitude (WGS84), in degrees * 1E7
    * @param lon             Longitude (WGS84), in degrees * 1E7
    * @param alt             AMSL, in meters
    * @param vel             target velocity (0,0,0) for unknown
    * @param acc             linear target acceleration (0,0,0) for unknown
    * @param attitudeQ       (1 0 0 0 for unknown)
    * @param rates           (0 0 0 for unknown)
    * @param positionCov     eph epv
    * @param customState     button states or switches of a tracker device
    */
  case class FollowTarget(
    timestamp: Long,
    estCapabilities: Int,
    lat: Int,
    lon: Int,
    alt: Float,
    vel: IndexedSeq[Float],
    acc: IndexedSeq[Float],
    attitudeQ: IndexedSeq[Float],
    rates: IndexedSeq[Float],
    positionCov: IndexedSeq[Float],
    customState: Long
  ) extends MAVMessage("FOLLOW_TARGET", systemId, componentId, protocolVersion) {
    set("timestamp", timestamp)
    set("est_capabilities", estCapabilities)
    set("lat", lat)
    set("lon", lon)
    set("alt", alt)
    set("vel", vel.map(float2Float).toArray)
    set("acc", acc.map(float2Float).toArray)
    set("attitude_q", attitudeQ.map(float2Float).toArray)
    set("rates", rates.map(float2Float).toArray)
    set("position_cov", positionCov.map(float2Float).toArray)
    set("custom_state", customState)

    override def toString: String =
      List(
        s"timestamp = $timestamp",
        s"estCapabilities = $estCapabilities",
        s"lat = $lat",
        s"lon = $lon",
        s"alt = $alt",
        s"vel = $vel",
        s"acc = $acc",
        s"attitudeQ = $attitudeQ",
        s"rates = $rates",
        s"positionCov = $positionCov",
        s"customState = $customState"
      ).mkString("FollowTarget(", ", ", ")")
  }

  /**
    * 146 CONTROL_SYSTEM_STATE
    *
    * The smoothed, monotonic system state used to feed the control loops of the system.
    *
    * @param timeUSec    Timestamp (micros since boot or Unix epoch)
    * @param xAcc        X acceleration in body frame
    * @param yAcc        Y acceleration in body frame
    * @param zAcc        Z acceleration in body frame
    * @param xVel        X velocity in body frame
    * @param yVel        Y velocity in body frame
    * @param zVel        Z velocity in body frame
    * @param xPos        X position in local frame
    * @param yPos        Y position in local frame
    * @param zPos        Z position in local frame
    * @param airspeed    Airspeed, set to -1 if unknown
    * @param velVariance Variance of body velocity estimate
    * @param posVariance Variance in local position
    * @param q           The attitude, represented as Quaternion
    * @param rollRate    Angular rate in roll axis
    * @param pitchRate   Angular rate in pitch axis
    * @param yawRate     Angular rate in yaw axis
    */
  case class ControlSystemState(
    timeUSec: Long,
    xAcc: Float,
    yAcc: Float,
    zAcc: Float,
    xVel: Float,
    yVel: Float,
    zVel: Float,
    xPos: Float,
    yPos: Float,
    zPos: Float,
    airspeed: Float,
    velVariance: IndexedSeq[Float],
    posVariance: IndexedSeq[Float],
    q: IndexedSeq[Float],
    rollRate: Float,
    pitchRate: Float,
    yawRate: Float
  ) extends MAVMessage("CONTROL_SYSTEM_STATE", systemId, componentId, protocolVersion) {
    set("time_usec", timeUSec)
    set("x_acc", xAcc)
    set("y_acc", yAcc)
    set("z_acc", zAcc)
    set("x_vel", xVel)
    set("y_vel", yVel)
    set("z_vel", zVel)
    set("x_pos", xPos)
    set("y_pos", yPos)
    set("z_pos", zPos)
    set("airspeed", airspeed)
    set("vel_variance", velVariance.map(float2Float).toArray)
    set("pos_variance", posVariance.map(float2Float).toArray)
    set("q", q.map(float2Float).toArray)
    set("roll_rate", rollRate)
    set("pitch_rate", pitchRate)
    set("yaw_rate", yawRate)

    override def toString: String =
      List(
        s"timeUSec = $timeUSec",
        s"xAcc = $xAcc",
        s"yAcc = $yAcc",
        s"zAcc = $zAcc",
        s"xVel = $xVel",
        s"yVel = $yVel",
        s"zVel = $zVel",
        s"xPos = $xPos",
        s"yPos = $yPos",
        s"zPos = $zPos",
        s"airspeed = $airspeed",
        s"velVariance = $velVariance",
        s"posVariance = $posVariance",
        s"q = $q",
        s"rollRate = $rollRate",
        s"pitchRate = $pitchRate",
        s"yawRate = $yawRate"
      ).mkString("ControlSystemState(", ", ", ")")
  }

  /**
    * 147 BATTERY_STATUS
    *
    * Battery information
    *
    * @param id               Battery ID
    * @param batteryFunction  Function of the battery
    * @param `type`           Type (chemistry) of the battery
    * @param temperature      Temperature of the battery in centi-degrees celsius. INT16_MAX for unknown temperature.
    * @param voltages         Battery voltage of cells, in millivolts (1 = 1 millivolt). Cells above the valid cell count for this battery should have the UINT16_MAX value.
    * @param currentBattery   Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current
    * @param currentConsumed  Consumed charge, in milliampere hours (1 = 1 mAh), -1: autopilot does not provide mAh consumption estimate
    * @param energyConsumed   Consumed energy, in HectoJoules (intergrated U*I*dt)  (1 = 100 Joule), -1: autopilot does not provide energy consumption estimate
    * @param batteryRemaining Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot does not estimate the remaining battery
    */
  case class BatteryStatus(
    id: Int,
    batteryFunction: MAVBatteryFunction,
    `type`: MAVBatteryType,
    temperature: Int,
    voltages: IndexedSeq[Int],
    currentBattery: Int,
    currentConsumed: Int,
    energyConsumed: Int,
    batteryRemaining: Int
  ) extends MAVMessage("BATTERY_STATUS", systemId, componentId, protocolVersion) {
    set("id", id)
    set("battery_function", batteryFunction.value)
    set("type", `type`.value)
    set("temperature", temperature)
    set("voltages", voltages.map(int2Integer).toArray)
    set("current_battery", currentBattery)
    set("current_consumed", currentConsumed)
    set("energy_consumed", energyConsumed)
    set("battery_remaining", batteryRemaining)

    override def toString: String =
      List(
        s"id = $id",
        s"batteryFunction = $batteryFunction",
        s"type = ${`type`}",
        s"temperature = $temperature",
        s"voltages = $voltages",
        s"currentBattery = $currentBattery",
        s"currentConsumed = $currentConsumed",
        s"energyConsumed = $energyConsumed",
        s"batteryRemaining = $batteryRemaining"
      ).mkString("BatteryStatus(", ", ", ")")
  }

  /**
    * 148 AUTOPILOT_VERSION
    *
    * Version and capability of autopilot software
    *
    * @param capabilities            bitmask of capabilities (see MAV_PROTOCOL_CAPABILITY enum)
    * @param flightSWVersion         Firmware version number
    * @param middlewareSWVersion     Middleware version number
    * @param oSSWVersion             Operating system version number
    * @param boardVersion            HW / board version (last 8 bytes should be silicon ID, if any)
    * @param flightCustomVersion     Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but should allow to identify the commit using the main version number even for very large code bases.
    * @param middlewareCustomVersion Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but should allow to identify the commit using the main version number even for very large code bases.
    * @param oSCustomVersion         Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but should allow to identify the commit using the main version number even for very large code bases.
    * @param vendorId                ID of the board vendor
    * @param productId               ID of the product
    * @param uid                     UID if provided by hardware (see uid2)
    * @param uid2                    UID if provided by hardware (supersedes the uid field. If this is non-zero, use this field, otherwise use uid)
    */
  case class AutopilotVersion(
    capabilities: Set[MAVProtocolCapability],
    flightSWVersion: Int,
    middlewareSWVersion: Int,
    oSSWVersion: Int,
    boardVersion: Int,
    flightCustomVersion: IndexedSeq[Int],
    middlewareCustomVersion: IndexedSeq[Int],
    oSCustomVersion: IndexedSeq[Int],
    vendorId: Int,
    productId: Int,
    uid: Long,
    uid2: IndexedSeq[Int]
  ) extends MAVMessage("AUTOPILOT_VERSION", systemId, componentId, protocolVersion) {
    set("capabilities", capabilities.foldLeft(0)(_ | _.value))
    set("flight_sw_version", flightSWVersion)
    set("middleware_sw_version", middlewareSWVersion)
    set("os_sw_version", oSSWVersion)
    set("board_version", boardVersion)
    set("flight_custom_version", flightCustomVersion.map(int2Integer).toArray)
    set("middleware_custom_version", middlewareCustomVersion.map(int2Integer).toArray)
    set("os_custom_version", oSCustomVersion.map(int2Integer).toArray)
    set("vendor_id", vendorId)
    set("product_id", productId)
    set("uid", uid)
    set("uid2", uid2.map(int2Integer).toArray)

    override def toString: String =
      List(
        s"capabilities = $capabilities",
        s"flightSWVersion = $flightSWVersion",
        s"middlewareSWVersion = $middlewareSWVersion",
        s"oSSWVersion = $oSSWVersion",
        s"boardVersion = $boardVersion",
        s"flightCustomVersion = $flightCustomVersion",
        s"middlewareCustomVersion = $middlewareCustomVersion",
        s"oSCustomVersion = $oSCustomVersion",
        s"vendorId = $vendorId",
        s"productId = $productId",
        s"uid = $uid",
        s"uid2 = $uid2"
      ).mkString("AutopilotVersion(", ", ", ")")
  }

  /**
    * 149 LANDING_TARGET
    *
    * The location of a landing area captured from a downward facing camera
    *
    * @param timeUSec      Timestamp (micros since boot or Unix epoch)
    * @param targetNum     The ID of the target if multiple targets are present
    * @param frame         MAV_FRAME enum specifying the whether the following feilds are earth-frame, body-frame, etc.
    * @param angleX        X-axis angular offset (in radians) of the target from the center of the image
    * @param angleY        Y-axis angular offset (in radians) of the target from the center of the image
    * @param distance      Distance to the target from the vehicle in meters
    * @param sizeX         Size in radians of target along x-axis
    * @param sizeY         Size in radians of target along y-axis
    * @param x             X Position of the landing target on MAV_FRAME
    * @param y             Y Position of the landing target on MAV_FRAME
    * @param z             Z Position of the landing target on MAV_FRAME
    * @param q             Quaternion of landing target orientation (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
    * @param `type`        LANDING_TARGET_TYPE enum specifying the type of landing target
    * @param positionValid Boolean indicating known position (1) or default unkown position (0), for validation of positioning of the landing target
    */
  case class LandingTarget(
    timeUSec: Long,
    targetNum: Int,
    frame: MAVFrame,
    angleX: Float,
    angleY: Float,
    distance: Float,
    sizeX: Float,
    sizeY: Float,
    x: Float,
    y: Float,
    z: Float,
    q: IndexedSeq[Float],
    `type`: LandingTargetType,
    positionValid: Int
  ) extends MAVMessage("LANDING_TARGET", systemId, componentId, protocolVersion) {
    set("time_usec", timeUSec)
    set("target_num", targetNum)
    set("frame", frame.value)
    set("angle_x", angleX)
    set("angle_y", angleY)
    set("distance", distance)
    set("size_x", sizeX)
    set("size_y", sizeY)
    set("x", x)
    set("y", y)
    set("z", z)
    set("q", q.map(float2Float).toArray)
    set("type", `type`.value)
    set("position_valid", positionValid)

    override def toString: String =
      List(
        s"timeUSec = $timeUSec",
        s"targetNum = $targetNum",
        s"frame = $frame",
        s"angleX = $angleX",
        s"angleY = $angleY",
        s"distance = $distance",
        s"sizeX = $sizeX",
        s"sizeY = $sizeY",
        s"x = $x",
        s"y = $y",
        s"z = $z",
        s"q = $q",
        s"type = ${`type`}",
        s"positionValid = $positionValid"
      ).mkString("LandingTarget(", ", ", ")")
  }

  /**
    * 230 ESTIMATOR_STATUS
    *
    * Estimator status message including flags, innovation test ratios and estimated accuracies. The flags message is an integer bitmask containing information on which EKF outputs are valid. See the ESTIMATOR_STATUS_FLAGS enum definition for further information. The innovaton test ratios show the magnitude of the sensor innovation divided by the innovation check threshold. Under normal operation the innovaton test ratios should be below 0.5 with occasional values up to 1.0. Values greater than 1.0 should be rare under normal operation and indicate that a measurement has been rejected by the filter. The user should be notified if an innovation test ratio greater than 1.0 is recorded. Notifications for values in the range between 0.5 and 1.0 should be optional and controllable by the user.
    *
    * @param timeUSec         Timestamp (micros since boot or Unix epoch)
    * @param flags            Integer bitmask indicating which EKF outputs are valid. See definition for ESTIMATOR_STATUS_FLAGS.
    * @param velRatio         Velocity innovation test ratio
    * @param posHorizRatio    Horizontal position innovation test ratio
    * @param posVertRatio     Vertical position innovation test ratio
    * @param magRatio         Magnetometer innovation test ratio
    * @param hAGLRatio        Height above terrain innovation test ratio
    * @param tASRatio         True airspeed innovation test ratio
    * @param posHorizAccuracy Horizontal position 1-STD accuracy relative to the EKF local origin (m)
    * @param posVertAccuracy  Vertical position 1-STD accuracy relative to the EKF local origin (m)
    */
  case class EstimatorStatus(
    timeUSec: Long,
    flags: Set[EstimatorStatusFlags],
    velRatio: Float,
    posHorizRatio: Float,
    posVertRatio: Float,
    magRatio: Float,
    hAGLRatio: Float,
    tASRatio: Float,
    posHorizAccuracy: Float,
    posVertAccuracy: Float
  ) extends MAVMessage("ESTIMATOR_STATUS", systemId, componentId, protocolVersion) {
    set("time_usec", timeUSec)
    set("flags", flags.foldLeft(0)(_ | _.value))
    set("vel_ratio", velRatio)
    set("pos_horiz_ratio", posHorizRatio)
    set("pos_vert_ratio", posVertRatio)
    set("mag_ratio", magRatio)
    set("hagl_ratio", hAGLRatio)
    set("tas_ratio", tASRatio)
    set("pos_horiz_accuracy", posHorizAccuracy)
    set("pos_vert_accuracy", posVertAccuracy)

    override def toString: String =
      List(
        s"timeUSec = $timeUSec",
        s"flags = $flags",
        s"velRatio = $velRatio",
        s"posHorizRatio = $posHorizRatio",
        s"posVertRatio = $posVertRatio",
        s"magRatio = $magRatio",
        s"hAGLRatio = $hAGLRatio",
        s"tASRatio = $tASRatio",
        s"posHorizAccuracy = $posHorizAccuracy",
        s"posVertAccuracy = $posVertAccuracy"
      ).mkString("EstimatorStatus(", ", ", ")")
  }

  /**
    * 231 WIND_COV
    *
    * @param timeUSec      Timestamp (micros since boot or Unix epoch)
    * @param windX         Wind in X (NED) direction in m/s
    * @param windY         Wind in Y (NED) direction in m/s
    * @param windZ         Wind in Z (NED) direction in m/s
    * @param varHoriz      Variability of the wind in XY. RMS of a 1 Hz lowpassed wind estimate.
    * @param varVert       Variability of the wind in Z. RMS of a 1 Hz lowpassed wind estimate.
    * @param windAlt       AMSL altitude (m) this measurement was taken at
    * @param horizAccuracy Horizontal speed 1-STD accuracy
    * @param vertAccuracy  Vertical speed 1-STD accuracy
    */
  case class WindCov(
    timeUSec: Long,
    windX: Float,
    windY: Float,
    windZ: Float,
    varHoriz: Float,
    varVert: Float,
    windAlt: Float,
    horizAccuracy: Float,
    vertAccuracy: Float
  ) extends MAVMessage("WIND_COV", systemId, componentId, protocolVersion) {
    set("time_usec", timeUSec)
    set("wind_x", windX)
    set("wind_y", windY)
    set("wind_z", windZ)
    set("var_horiz", varHoriz)
    set("var_vert", varVert)
    set("wind_alt", windAlt)
    set("horiz_accuracy", horizAccuracy)
    set("vert_accuracy", vertAccuracy)

    override def toString: String =
      List(
        s"timeUSec = $timeUSec",
        s"windX = $windX",
        s"windY = $windY",
        s"windZ = $windZ",
        s"varHoriz = $varHoriz",
        s"varVert = $varVert",
        s"windAlt = $windAlt",
        s"horizAccuracy = $horizAccuracy",
        s"vertAccuracy = $vertAccuracy"
      ).mkString("WindCov(", ", ", ")")
  }

  /**
    * 232 GPS_INPUT
    *
    * GPS sensor input message.  This is a raw sensor value sent by the GPS. This is NOT the global position estimate of the sytem.
    *
    * @param timeUSec          Timestamp (micros since boot or Unix epoch)
    * @param gPSId             ID of the GPS for multiple GPS inputs
    * @param ignoreFlags       Flags indicating which fields to ignore (see GPS_INPUT_IGNORE_FLAGS enum).  All other fields must be provided.
    * @param timeWeekMS        GPS time (milliseconds from start of GPS week)
    * @param timeWeek          GPS week number
    * @param fixType           0-1: no fix, 2: 2D fix, 3: 3D fix. 4: 3D with DGPS. 5: 3D with RTK
    * @param lat               Latitude (WGS84), in degrees * 1E7
    * @param lon               Longitude (WGS84), in degrees * 1E7
    * @param alt               Altitude (AMSL, not WGS84), in m (positive for up)
    * @param hDOP              GPS HDOP horizontal dilution of position in m
    * @param vDOP              GPS VDOP vertical dilution of position in m
    * @param vN                GPS velocity in m/s in NORTH direction in earth-fixed NED frame
    * @param vE                GPS velocity in m/s in EAST direction in earth-fixed NED frame
    * @param vD                GPS velocity in m/s in DOWN direction in earth-fixed NED frame
    * @param speedAccuracy     GPS speed accuracy in m/s
    * @param horizAccuracy     GPS horizontal accuracy in m
    * @param vertAccuracy      GPS vertical accuracy in m
    * @param satellitesVisible Number of satellites visible.
    */
  case class GPSInput(
    timeUSec: Long,
    gPSId: Int,
    ignoreFlags: Set[GPSInputIgnoreFlags],
    timeWeekMS: Int,
    timeWeek: Int,
    fixType: Int,
    lat: Int,
    lon: Int,
    alt: Float,
    hDOP: Float,
    vDOP: Float,
    vN: Float,
    vE: Float,
    vD: Float,
    speedAccuracy: Float,
    horizAccuracy: Float,
    vertAccuracy: Float,
    satellitesVisible: Int
  ) extends MAVMessage("GPS_INPUT", systemId, componentId, protocolVersion) {
    set("time_usec", timeUSec)
    set("gps_id", gPSId)
    set("ignore_flags", ignoreFlags.foldLeft(0)(_ | _.value))
    set("time_week_ms", timeWeekMS)
    set("time_week", timeWeek)
    set("fix_type", fixType)
    set("lat", lat)
    set("lon", lon)
    set("alt", alt)
    set("hdop", hDOP)
    set("vdop", vDOP)
    set("vn", vN)
    set("ve", vE)
    set("vd", vD)
    set("speed_accuracy", speedAccuracy)
    set("horiz_accuracy", horizAccuracy)
    set("vert_accuracy", vertAccuracy)
    set("satellites_visible", satellitesVisible)

    override def toString: String =
      List(
        s"timeUSec = $timeUSec",
        s"gPSId = $gPSId",
        s"ignoreFlags = $ignoreFlags",
        s"timeWeekMS = $timeWeekMS",
        s"timeWeek = $timeWeek",
        s"fixType = $fixType",
        s"lat = $lat",
        s"lon = $lon",
        s"alt = $alt",
        s"hDOP = $hDOP",
        s"vDOP = $vDOP",
        s"vN = $vN",
        s"vE = $vE",
        s"vD = $vD",
        s"speedAccuracy = $speedAccuracy",
        s"horizAccuracy = $horizAccuracy",
        s"vertAccuracy = $vertAccuracy",
        s"satellitesVisible = $satellitesVisible"
      ).mkString("GPSInput(", ", ", ")")
  }

  /**
    * 233 GPS_RTCM_DATA
    *
    * RTCM message for injecting into the onboard GPS (used for DGPS)
    *
    * @param flags LSB: 1 means message is fragmented, next 2 bits are the fragment ID, the remaining 5 bits are used for the sequence ID. Messages are only to be flushed to the GPS when the entire message has been reconstructed on the autopilot. The fragment ID specifies which order the fragments should be assembled into a buffer, while the sequence ID is used to detect a mismatch between different buffers. The buffer is considered fully reconstructed when either all 4 fragments are present, or all the fragments before the first fragment with a non full payload is received. This management is used to ensure that normal GPS operation doesn't corrupt RTCM data, and to recover from a unreliable transport delivery order.
    * @param len   data length
    * @param data  RTCM message (may be fragmented)
    */
  case class GPSRTCMData(
    flags: Int,
    len: Int,
    data: IndexedSeq[Int]
  ) extends MAVMessage("GPS_RTCM_DATA", systemId, componentId, protocolVersion) {
    set("flags", flags)
    set("len", len)
    set("data", data.map(int2Integer).toArray)

    override def toString: String =
      List(
        s"flags = $flags",
        s"len = $len",
        s"data = $data"
      ).mkString("GPSRTCMData(", ", ", ")")
  }

  /**
    * 234 HIGH_LATENCY
    *
    * Message appropriate for high latency connections like Iridium
    *
    * @param baseMode         System mode bitfield, see MAV_MODE_FLAG ENUM in mavlink/include/mavlink_types.h
    * @param customMode       A bitfield for use for autopilot-specific flags.
    * @param landedState      The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown.
    * @param roll             roll (centidegrees)
    * @param pitch            pitch (centidegrees)
    * @param heading          heading (centidegrees)
    * @param throttle         throttle (percentage)
    * @param headingSp        heading setpoint (centidegrees)
    * @param latitude         Latitude, expressed as degrees * 1E7
    * @param longitude        Longitude, expressed as degrees * 1E7
    * @param altitudeAMSL     Altitude above mean sea level (meters)
    * @param altitudeSp       Altitude setpoint relative to the home position (meters)
    * @param airspeed         airspeed (m/s)
    * @param airspeedSp       airspeed setpoint (m/s)
    * @param groundSpeed      groundspeed (m/s)
    * @param climbRate        climb rate (m/s)
    * @param gPSNsat          Number of satellites visible. If unknown, set to 255
    * @param gPSFixType       See the GPS_FIX_TYPE enum.
    * @param batteryRemaining Remaining battery (percentage)
    * @param temperature      Autopilot temperature (degrees C)
    * @param temperatureAir   Air temperature (degrees C) from airspeed sensor
    * @param failsafe         failsafe (each bit represents a failsafe where 0=ok, 1=failsafe active (bit0:RC, bit1:batt, bit2:GPS, bit3:GCS, bit4:fence)
    * @param wpNum            current waypoint number
    * @param wpDistance       distance to target (meters)
    */
  case class HighLatency(
    baseMode: Set[MAVModeFlag],
    customMode: Int,
    landedState: MAVLandedState,
    roll: Int,
    pitch: Int,
    heading: Int,
    throttle: Int,
    headingSp: Int,
    latitude: Int,
    longitude: Int,
    altitudeAMSL: Int,
    altitudeSp: Int,
    airspeed: Int,
    airspeedSp: Int,
    groundSpeed: Int,
    climbRate: Int,
    gPSNsat: Int,
    gPSFixType: GPSFixType,
    batteryRemaining: Int,
    temperature: Int,
    temperatureAir: Int,
    failsafe: Int,
    wpNum: Int,
    wpDistance: Int
  ) extends MAVMessage("HIGH_LATENCY", systemId, componentId, protocolVersion) {
    set("base_mode", baseMode.foldLeft(0)(_ | _.value))
    set("custom_mode", customMode)
    set("landed_state", landedState.value)
    set("roll", roll)
    set("pitch", pitch)
    set("heading", heading)
    set("throttle", throttle)
    set("heading_sp", headingSp)
    set("latitude", latitude)
    set("longitude", longitude)
    set("altitude_amsl", altitudeAMSL)
    set("altitude_sp", altitudeSp)
    set("airspeed", airspeed)
    set("airspeed_sp", airspeedSp)
    set("groundspeed", groundSpeed)
    set("climb_rate", climbRate)
    set("gps_nsat", gPSNsat)
    set("gps_fix_type", gPSFixType.value)
    set("battery_remaining", batteryRemaining)
    set("temperature", temperature)
    set("temperature_air", temperatureAir)
    set("failsafe", failsafe)
    set("wp_num", wpNum)
    set("wp_distance", wpDistance)

    override def toString: String =
      List(
        s"baseMode = $baseMode",
        s"customMode = $customMode",
        s"landedState = $landedState",
        s"roll = $roll",
        s"pitch = $pitch",
        s"heading = $heading",
        s"throttle = $throttle",
        s"headingSp = $headingSp",
        s"latitude = $latitude",
        s"longitude = $longitude",
        s"altitudeAMSL = $altitudeAMSL",
        s"altitudeSp = $altitudeSp",
        s"airspeed = $airspeed",
        s"airspeedSp = $airspeedSp",
        s"groundSpeed = $groundSpeed",
        s"climbRate = $climbRate",
        s"gPSNsat = $gPSNsat",
        s"gPSFixType = $gPSFixType",
        s"batteryRemaining = $batteryRemaining",
        s"temperature = $temperature",
        s"temperatureAir = $temperatureAir",
        s"failsafe = $failsafe",
        s"wpNum = $wpNum",
        s"wpDistance = $wpDistance"
      ).mkString("HighLatency(", ", ", ")")
  }

  /**
    * 241 VIBRATION
    *
    * Vibration levels and accelerometer clipping
    *
    * @param timeUSec   Timestamp (micros since boot or Unix epoch)
    * @param vibrationX Vibration levels on X-axis
    * @param vibrationY Vibration levels on Y-axis
    * @param vibrationZ Vibration levels on Z-axis
    * @param clipping0  first accelerometer clipping count
    * @param clipping1  second accelerometer clipping count
    * @param clipping2  third accelerometer clipping count
    */
  case class Vibration(
    timeUSec: Long,
    vibrationX: Float,
    vibrationY: Float,
    vibrationZ: Float,
    clipping0: Int,
    clipping1: Int,
    clipping2: Int
  ) extends MAVMessage("VIBRATION", systemId, componentId, protocolVersion) {
    set("time_usec", timeUSec)
    set("vibration_x", vibrationX)
    set("vibration_y", vibrationY)
    set("vibration_z", vibrationZ)
    set("clipping_0", clipping0)
    set("clipping_1", clipping1)
    set("clipping_2", clipping2)

    override def toString: String =
      List(
        s"timeUSec = $timeUSec",
        s"vibrationX = $vibrationX",
        s"vibrationY = $vibrationY",
        s"vibrationZ = $vibrationZ",
        s"clipping0 = $clipping0",
        s"clipping1 = $clipping1",
        s"clipping2 = $clipping2"
      ).mkString("Vibration(", ", ", ")")
  }

  /**
    * 242 HOME_POSITION
    *
    * This message can be requested by sending the MAV_CMD_GET_HOME_POSITION command. The position the system will return to and land on. The position is set automatically by the system during the takeoff in case it was not explicitely set by the operator before or after. The position the system will return to and land on. The global and local positions encode the position in the respective coordinate frames, while the q parameter encodes the orientation of the surface. Under normal conditions it describes the heading and terrain slope, which can be used by the aircraft to adjust the approach. The approach 3D vector describes the point to which the system should fly in normal flight mode and then perform a landing sequence along the vector.
    *
    * @param latitude  Latitude (WGS84), in degrees * 1E7
    * @param longitude Longitude (WGS84, in degrees * 1E7
    * @param altitude  Altitude (AMSL), in meters * 1000 (positive for up)
    * @param x         Local X position of this position in the local coordinate frame
    * @param y         Local Y position of this position in the local coordinate frame
    * @param z         Local Z position of this position in the local coordinate frame
    * @param q         World to surface normal and heading transformation of the takeoff position. Used to indicate the heading and slope of the ground
    * @param approachX Local X position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone.
    * @param approachY Local Y position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone.
    * @param approachZ Local Z position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone.
    * @param timeUSec  Timestamp (microseconds since UNIX epoch or microseconds since system boot)
    */
  case class HomePosition(
    latitude: Int,
    longitude: Int,
    altitude: Int,
    x: Float,
    y: Float,
    z: Float,
    q: IndexedSeq[Float],
    approachX: Float,
    approachY: Float,
    approachZ: Float,
    timeUSec: Long
  ) extends MAVMessage("HOME_POSITION", systemId, componentId, protocolVersion) {
    set("latitude", latitude)
    set("longitude", longitude)
    set("altitude", altitude)
    set("x", x)
    set("y", y)
    set("z", z)
    set("q", q.map(float2Float).toArray)
    set("approach_x", approachX)
    set("approach_y", approachY)
    set("approach_z", approachZ)
    set("time_usec", timeUSec)

    override def toString: String =
      List(
        s"latitude = $latitude",
        s"longitude = $longitude",
        s"altitude = $altitude",
        s"x = $x",
        s"y = $y",
        s"z = $z",
        s"q = $q",
        s"approachX = $approachX",
        s"approachY = $approachY",
        s"approachZ = $approachZ",
        s"timeUSec = $timeUSec"
      ).mkString("HomePosition(", ", ", ")")
  }

  /**
    * 243 SET_HOME_POSITION
    *
    * The position the system will return to and land on. The position is set automatically by the system during the takeoff in case it was not explicitely set by the operator before or after. The global and local positions encode the position in the respective coordinate frames, while the q parameter encodes the orientation of the surface. Under normal conditions it describes the heading and terrain slope, which can be used by the aircraft to adjust the approach. The approach 3D vector describes the point to which the system should fly in normal flight mode and then perform a landing sequence along the vector.
    *
    * @param targetSystem System ID.
    * @param latitude     Latitude (WGS84), in degrees * 1E7
    * @param longitude    Longitude (WGS84, in degrees * 1E7
    * @param altitude     Altitude (AMSL), in meters * 1000 (positive for up)
    * @param x            Local X position of this position in the local coordinate frame
    * @param y            Local Y position of this position in the local coordinate frame
    * @param z            Local Z position of this position in the local coordinate frame
    * @param q            World to surface normal and heading transformation of the takeoff position. Used to indicate the heading and slope of the ground
    * @param approachX    Local X position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone.
    * @param approachY    Local Y position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone.
    * @param approachZ    Local Z position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone.
    * @param timeUSec     Timestamp (microseconds since UNIX epoch or microseconds since system boot)
    */
  case class SetHomePosition(
    targetSystem: Int,
    latitude: Int,
    longitude: Int,
    altitude: Int,
    x: Float,
    y: Float,
    z: Float,
    q: IndexedSeq[Float],
    approachX: Float,
    approachY: Float,
    approachZ: Float,
    timeUSec: Long
  ) extends MAVMessage("SET_HOME_POSITION", systemId, componentId, protocolVersion) {
    set("target_system", targetSystem)
    set("latitude", latitude)
    set("longitude", longitude)
    set("altitude", altitude)
    set("x", x)
    set("y", y)
    set("z", z)
    set("q", q.map(float2Float).toArray)
    set("approach_x", approachX)
    set("approach_y", approachY)
    set("approach_z", approachZ)
    set("time_usec", timeUSec)

    override def toString: String =
      List(
        s"targetSystem = $targetSystem",
        s"latitude = $latitude",
        s"longitude = $longitude",
        s"altitude = $altitude",
        s"x = $x",
        s"y = $y",
        s"z = $z",
        s"q = $q",
        s"approachX = $approachX",
        s"approachY = $approachY",
        s"approachZ = $approachZ",
        s"timeUSec = $timeUSec"
      ).mkString("SetHomePosition(", ", ", ")")
  }

  /**
    * 244 MESSAGE_INTERVAL
    *
    * This interface replaces DATA_STREAM
    *
    * @param messageId  The ID of the requested MAVLink message. v1.0 is limited to 254 messages.
    * @param intervalUS The interval between two messages, in microseconds. A value of -1 indicates this stream is disabled, 0 indicates it is not available, > 0 indicates the interval at which it is sent.
    */
  case class MessageInterval(
    messageId: Int,
    intervalUS: Int
  ) extends MAVMessage("MESSAGE_INTERVAL", systemId, componentId, protocolVersion) {
    set("message_id", messageId)
    set("interval_us", intervalUS)

    override def toString: String =
      List(
        s"messageId = $messageId",
        s"intervalUS = $intervalUS"
      ).mkString("MessageInterval(", ", ", ")")
  }

  /**
    * 245 EXTENDED_SYS_STATE
    *
    * Provides state for additional features
    *
    * @param vTOLState   The VTOL state if applicable. Is set to MAV_VTOL_STATE_UNDEFINED if UAV is not in VTOL configuration.
    * @param landedState The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown.
    */
  case class ExtendedSysState(
    vTOLState: MAVVTOLState,
    landedState: MAVLandedState
  ) extends MAVMessage("EXTENDED_SYS_STATE", systemId, componentId, protocolVersion) {
    set("vtol_state", vTOLState.value)
    set("landed_state", landedState.value)

    override def toString: String =
      List(
        s"vTOLState = $vTOLState",
        s"landedState = $landedState"
      ).mkString("ExtendedSysState(", ", ", ")")
  }

  /**
    * 246 ADSB_VEHICLE
    *
    * The location and information of an ADSB vehicle
    *
    * @param icaoAddress  ICAO address
    * @param lat          Latitude, expressed as degrees * 1E7
    * @param lon          Longitude, expressed as degrees * 1E7
    * @param altitudeType Type from ADSB_ALTITUDE_TYPE enum
    * @param altitude     Altitude(ASL) in millimeters
    * @param heading      Course over ground in centidegrees
    * @param horVelocity  The horizontal velocity in centimeters/second
    * @param verVelocity  The vertical velocity in centimeters/second, positive is up
    * @param callsign     The callsign, 8+null
    * @param emitterType  Type from ADSB_EMITTER_TYPE enum
    * @param tslc         Time since last communication in seconds
    * @param flags        Flags to indicate various statuses including valid data fields
    * @param squawk       Squawk code
    */
  case class ADSBVehicle(
    icaoAddress: Int,
    lat: Int,
    lon: Int,
    altitudeType: ADSBAltitudeType,
    altitude: Int,
    heading: Int,
    horVelocity: Int,
    verVelocity: Int,
    callsign: String,
    emitterType: ADSBEmitterType,
    tslc: Int,
    flags: Set[ADSBFlags],
    squawk: Int
  ) extends MAVMessage("ADSB_VEHICLE", systemId, componentId, protocolVersion) {
    set("ICAO_address", icaoAddress)
    set("lat", lat)
    set("lon", lon)
    set("altitude_type", altitudeType.value)
    set("altitude", altitude)
    set("heading", heading)
    set("hor_velocity", horVelocity)
    set("ver_velocity", verVelocity)
    set("callsign", callsign)
    set("emitter_type", emitterType.value)
    set("tslc", tslc)
    set("flags", flags.foldLeft(0)(_ | _.value))
    set("squawk", squawk)

    override def toString: String =
      List(
        s"icaoAddress = $icaoAddress",
        s"lat = $lat",
        s"lon = $lon",
        s"altitudeType = $altitudeType",
        s"altitude = $altitude",
        s"heading = $heading",
        s"horVelocity = $horVelocity",
        s"verVelocity = $verVelocity",
        s"callsign = $callsign",
        s"emitterType = $emitterType",
        s"tslc = $tslc",
        s"flags = $flags",
        s"squawk = $squawk"
      ).mkString("ADSBVehicle(", ", ", ")")
  }

  /**
    * 247 COLLISION
    *
    * Information about a potential collision
    *
    * @param src                    Collision data source
    * @param id                     Unique identifier, domain based on src field
    * @param action                 Action that is being taken to avoid this collision
    * @param threatLevel            How concerned the aircraft is about this collision
    * @param timeToMinimumDelta     Estimated time until collision occurs (seconds)
    * @param altitudeMinimumDelta   Closest vertical distance in meters between vehicle and object
    * @param horizontalMinimumDelta Closest horizontal distance in meteres between vehicle and object
    */
  case class Collision(
    src: MAVCollisionSrc,
    id: Int,
    action: MAVCollisionAction,
    threatLevel: MAVCollisionThreatLevel,
    timeToMinimumDelta: Float,
    altitudeMinimumDelta: Float,
    horizontalMinimumDelta: Float
  ) extends MAVMessage("COLLISION", systemId, componentId, protocolVersion) {
    set("src", src.value)
    set("id", id)
    set("action", action.value)
    set("threat_level", threatLevel.value)
    set("time_to_minimum_delta", timeToMinimumDelta)
    set("altitude_minimum_delta", altitudeMinimumDelta)
    set("horizontal_minimum_delta", horizontalMinimumDelta)

    override def toString: String =
      List(
        s"src = $src",
        s"id = $id",
        s"action = $action",
        s"threatLevel = $threatLevel",
        s"timeToMinimumDelta = $timeToMinimumDelta",
        s"altitudeMinimumDelta = $altitudeMinimumDelta",
        s"horizontalMinimumDelta = $horizontalMinimumDelta"
      ).mkString("Collision(", ", ", ")")
  }

  /**
    * 248 V2_EXTENSION
    *
    * Message implementing parts of the V2 payload specs in V1 frames for transitional support.
    *
    * @param targetNetwork   Network ID (0 for broadcast)
    * @param targetSystem    System ID (0 for broadcast)
    * @param targetComponent Component ID (0 for broadcast)
    * @param messageType     A code that identifies the software component that understands this message (analogous to usb device classes or mime type strings).  If this code is less than 32768, it is considered a 'registered' protocol extension and the corresponding entry should be added to https://github.com/mavlink/mavlink/extension-message-ids.xml.  Software creators can register blocks of message IDs as needed (useful for GCS specific metadata, etc...). Message_types greater than 32767 are considered local experiments and should not be checked in to any widely distributed codebase.
    * @param payload         Variable length payload. The length is defined by the remaining message length when subtracting the header and other fields.  The entire content of this block is opaque unless you understand any the encoding message_type.  The particular encoding used can be extension specific and might not always be documented as part of the mavlink specification.
    */
  case class V2Extension(
    targetNetwork: Int,
    targetSystem: Int,
    targetComponent: Int,
    messageType: Int,
    payload: IndexedSeq[Int]
  ) extends MAVMessage("V2_EXTENSION", systemId, componentId, protocolVersion) {
    set("target_network", targetNetwork)
    set("target_system", targetSystem)
    set("target_component", targetComponent)
    set("message_type", messageType)
    set("payload", payload.map(int2Integer).toArray)

    override def toString: String =
      List(
        s"targetNetwork = $targetNetwork",
        s"targetSystem = $targetSystem",
        s"targetComponent = $targetComponent",
        s"messageType = $messageType",
        s"payload = $payload"
      ).mkString("V2Extension(", ", ", ")")
  }

  /**
    * 249 MEMORY_VECT
    *
    * Send raw controller memory. The use of this message is discouraged for normal packets, but a quite efficient way for testing new messages and getting experimental debug output.
    *
    * @param address Starting address of the debug variables
    * @param ver     Version code of the type variable. 0=unknown, type ignored and assumed int16_t. 1=as below
    * @param `type`  Type code of the memory variables. for ver = 1: 0=16 x int16_t, 1=16 x uint16_t, 2=16 x Q15, 3=16 x 1Q14
    * @param value   Memory contents at specified address
    */
  case class MemoryVect(
    address: Int,
    ver: Int,
    `type`: Int,
    value: IndexedSeq[Int]
  ) extends MAVMessage("MEMORY_VECT", systemId, componentId, protocolVersion) {
    set("address", address)
    set("ver", ver)
    set("type", `type`)
    set("value", value.map(int2Integer).toArray)

    override def toString: String =
      List(
        s"address = $address",
        s"ver = $ver",
        s"type = ${`type`}",
        s"value = $value"
      ).mkString("MemoryVect(", ", ", ")")
  }

  /**
    * 250 DEBUG_VECT
    *
    * @param name     Name
    * @param timeUSec Timestamp
    * @param x        x
    * @param y        y
    * @param z        z
    */
  case class DebugVect(
    name: String,
    timeUSec: Long,
    x: Float,
    y: Float,
    z: Float
  ) extends MAVMessage("DEBUG_VECT", systemId, componentId, protocolVersion) {
    set("name", name)
    set("time_usec", timeUSec)
    set("x", x)
    set("y", y)
    set("z", z)

    override def toString: String =
      List(
        s"name = $name",
        s"timeUSec = $timeUSec",
        s"x = $x",
        s"y = $y",
        s"z = $z"
      ).mkString("DebugVect(", ", ", ")")
  }

  /**
    * 251 NAMED_VALUE_FLOAT
    *
    * Send a key-value pair as float. The use of this message is discouraged for normal packets, but a quite efficient way for testing new messages and getting experimental debug output.
    *
    * @param timeBootMS Timestamp (milliseconds since system boot)
    * @param name       Name of the debug variable
    * @param value      Floating point value
    */
  case class NamedValueFloat(
    timeBootMS: Int,
    name: String,
    value: Float
  ) extends MAVMessage("NAMED_VALUE_FLOAT", systemId, componentId, protocolVersion) {
    set("time_boot_ms", timeBootMS)
    set("name", name)
    set("value", value)

    override def toString: String =
      List(
        s"timeBootMS = $timeBootMS",
        s"name = $name",
        s"value = $value"
      ).mkString("NamedValueFloat(", ", ", ")")
  }

  /**
    * 252 NAMED_VALUE_INT
    *
    * Send a key-value pair as integer. The use of this message is discouraged for normal packets, but a quite efficient way for testing new messages and getting experimental debug output.
    *
    * @param timeBootMS Timestamp (milliseconds since system boot)
    * @param name       Name of the debug variable
    * @param value      Signed integer value
    */
  case class NamedValueInt(
    timeBootMS: Int,
    name: String,
    value: Int
  ) extends MAVMessage("NAMED_VALUE_INT", systemId, componentId, protocolVersion) {
    set("time_boot_ms", timeBootMS)
    set("name", name)
    set("value", value)

    override def toString: String =
      List(
        s"timeBootMS = $timeBootMS",
        s"name = $name",
        s"value = $value"
      ).mkString("NamedValueInt(", ", ", ")")
  }

  /**
    * 253 STATUSTEXT
    *
    * Status text message. These messages are printed in yellow in the COMM console of QGroundControl. WARNING: They consume quite some bandwidth, so use only for important status and error messages. If implemented wisely, these messages are buffered on the MCU and sent only at a limited rate (e.g. 10 Hz).
    *
    * @param severity Severity of status. Relies on the definitions within RFC-5424. See enum MAV_SEVERITY.
    * @param text     Status text message, without null termination character
    */
  case class Statustext(
    severity: MAVSeverity,
    text: String
  ) extends MAVMessage("STATUSTEXT", systemId, componentId, protocolVersion) {
    set("severity", severity.value)
    set("text", text)

    override def toString: String =
      List(
        s"severity = $severity",
        s"text = $text"
      ).mkString("Statustext(", ", ", ")")
  }

  /**
    * 254 DEBUG
    *
    * Send a debug value. The index is used to discriminate between values. These values show up in the plot of QGroundControl as DEBUG N.
    *
    * @param timeBootMS Timestamp (milliseconds since system boot)
    * @param ind        index of debug variable
    * @param value      DEBUG value
    */
  case class Debug(
    timeBootMS: Int,
    ind: Int,
    value: Float
  ) extends MAVMessage("DEBUG", systemId, componentId, protocolVersion) {
    set("time_boot_ms", timeBootMS)
    set("ind", ind)
    set("value", value)

    override def toString: String =
      List(
        s"timeBootMS = $timeBootMS",
        s"ind = $ind",
        s"value = $value"
      ).mkString("Debug(", ", ", ")")
  }

  /**
    * 256 SETUP_SIGNING
    *
    * Setup a MAVLink2 signing key. If called with secret_key of all zero and zero initial_timestamp will disable signing
    *
    * @param targetSystem     system id of the target
    * @param targetComponent  component ID of the target
    * @param secretKey        signing key
    * @param initialTimestamp initial timestamp
    */
  case class SetupSigning(
    targetSystem: Int,
    targetComponent: Int,
    secretKey: IndexedSeq[Int],
    initialTimestamp: Long
  ) extends MAVMessage("SETUP_SIGNING", systemId, componentId, protocolVersion) {
    set("target_system", targetSystem)
    set("target_component", targetComponent)
    set("secret_key", secretKey.map(int2Integer).toArray)
    set("initial_timestamp", initialTimestamp)

    override def toString: String =
      List(
        s"targetSystem = $targetSystem",
        s"targetComponent = $targetComponent",
        s"secretKey = $secretKey",
        s"initialTimestamp = $initialTimestamp"
      ).mkString("SetupSigning(", ", ", ")")
  }

  /**
    * 257 BUTTON_CHANGE
    *
    * Report button state change
    *
    * @param timeBootMS   Timestamp (milliseconds since system boot)
    * @param lastChangeMS Time of last change of button state
    * @param state        Bitmap state of buttons
    */
  case class ButtonChange(
    timeBootMS: Int,
    lastChangeMS: Int,
    state: Int
  ) extends MAVMessage("BUTTON_CHANGE", systemId, componentId, protocolVersion) {
    set("time_boot_ms", timeBootMS)
    set("last_change_ms", lastChangeMS)
    set("state", state)

    override def toString: String =
      List(
        s"timeBootMS = $timeBootMS",
        s"lastChangeMS = $lastChangeMS",
        s"state = $state"
      ).mkString("ButtonChange(", ", ", ")")
  }

  /**
    * 258 PLAY_TUNE
    *
    * Control vehicle tone generation (buzzer)
    *
    * @param targetSystem    System ID
    * @param targetComponent Component ID
    * @param tune            tune in board specific format
    */
  case class PlayTune(
    targetSystem: Int,
    targetComponent: Int,
    tune: String
  ) extends MAVMessage("PLAY_TUNE", systemId, componentId, protocolVersion) {
    set("target_system", targetSystem)
    set("target_component", targetComponent)
    set("tune", tune)

    override def toString: String =
      List(
        s"targetSystem = $targetSystem",
        s"targetComponent = $targetComponent",
        s"tune = $tune"
      ).mkString("PlayTune(", ", ", ")")
  }

  /**
    * 259 CAMERA_INFORMATION
    *
    * WIP: Information about a camera
    *
    * @param timeBootMS           Timestamp (milliseconds since system boot)
    * @param vendorName           Name of the camera vendor
    * @param modelName            Name of the camera model
    * @param firmwareVersion      Version of the camera firmware (v << 24 & 0xff = Dev, v << 16 & 0xff = Patch, v << 8 & 0xff = Minor, v & 0xff = Major)
    * @param focalLength          Focal length in mm
    * @param sensorSizeH          Image sensor size horizontal in mm
    * @param sensorSizeV          Image sensor size vertical in mm
    * @param resolutionH          Image resolution in pixels horizontal
    * @param resolutionV          Image resolution in pixels vertical
    * @param lensId               Reserved for a lens ID
    * @param flags                CAMERA_CAP_FLAGS enum flags (bitmap) describing camera capabilities.
    * @param camDefinitionVersion Camera definition version (iteration)
    * @param camDefinitionURI     Camera definition URI (if any, otherwise only basic functions will be available).
    */
  case class CameraInformation(
    timeBootMS: Int,
    vendorName: IndexedSeq[Int],
    modelName: IndexedSeq[Int],
    firmwareVersion: Int,
    focalLength: Float,
    sensorSizeH: Float,
    sensorSizeV: Float,
    resolutionH: Int,
    resolutionV: Int,
    lensId: Int,
    flags: CameraCapFlags,
    camDefinitionVersion: Int,
    camDefinitionURI: String
  ) extends MAVMessage("CAMERA_INFORMATION", systemId, componentId, protocolVersion) {
    set("time_boot_ms", timeBootMS)
    set("vendor_name", vendorName.map(int2Integer).toArray)
    set("model_name", modelName.map(int2Integer).toArray)
    set("firmware_version", firmwareVersion)
    set("focal_length", focalLength)
    set("sensor_size_h", sensorSizeH)
    set("sensor_size_v", sensorSizeV)
    set("resolution_h", resolutionH)
    set("resolution_v", resolutionV)
    set("lens_id", lensId)
    set("flags", flags.value)
    set("cam_definition_version", camDefinitionVersion)
    set("cam_definition_uri", camDefinitionURI)

    override def toString: String =
      List(
        s"timeBootMS = $timeBootMS",
        s"vendorName = $vendorName",
        s"modelName = $modelName",
        s"firmwareVersion = $firmwareVersion",
        s"focalLength = $focalLength",
        s"sensorSizeH = $sensorSizeH",
        s"sensorSizeV = $sensorSizeV",
        s"resolutionH = $resolutionH",
        s"resolutionV = $resolutionV",
        s"lensId = $lensId",
        s"flags = $flags",
        s"camDefinitionVersion = $camDefinitionVersion",
        s"camDefinitionURI = $camDefinitionURI"
      ).mkString("CameraInformation(", ", ", ")")
  }

  /**
    * 260 CAMERA_SETTINGS
    *
    * WIP: Settings of a camera, can be requested using MAV_CMD_REQUEST_CAMERA_SETTINGS.
    *
    * @param timeBootMS Timestamp (milliseconds since system boot)
    * @param modeId     Camera mode (CAMERA_MODE)
    */
  case class CameraSettings(
    timeBootMS: Int,
    modeId: CameraMode
  ) extends MAVMessage("CAMERA_SETTINGS", systemId, componentId, protocolVersion) {
    set("time_boot_ms", timeBootMS)
    set("mode_id", modeId.value)

    override def toString: String =
      List(
        s"timeBootMS = $timeBootMS",
        s"modeId = $modeId"
      ).mkString("CameraSettings(", ", ", ")")
  }

  /**
    * 261 STORAGE_INFORMATION
    *
    * WIP: Information about a storage medium.
    *
    * @param timeBootMS        Timestamp (milliseconds since system boot)
    * @param storageId         Storage ID (1 for first, 2 for second, etc.)
    * @param storageCount      Number of storage devices
    * @param status            Status of storage (0 not available, 1 unformatted, 2 formatted)
    * @param totalCapacity     Total capacity in MiB
    * @param usedCapacity      Used capacity in MiB
    * @param availableCapacity Available capacity in MiB
    * @param readSpeed         Read speed in MiB/s
    * @param writeSpeed        Write speed in MiB/s
    */
  case class StorageInformation(
    timeBootMS: Int,
    storageId: Int,
    storageCount: Int,
    status: Int,
    totalCapacity: Float,
    usedCapacity: Float,
    availableCapacity: Float,
    readSpeed: Float,
    writeSpeed: Float
  ) extends MAVMessage("STORAGE_INFORMATION", systemId, componentId, protocolVersion) {
    set("time_boot_ms", timeBootMS)
    set("storage_id", storageId)
    set("storage_count", storageCount)
    set("status", status)
    set("total_capacity", totalCapacity)
    set("used_capacity", usedCapacity)
    set("available_capacity", availableCapacity)
    set("read_speed", readSpeed)
    set("write_speed", writeSpeed)

    override def toString: String =
      List(
        s"timeBootMS = $timeBootMS",
        s"storageId = $storageId",
        s"storageCount = $storageCount",
        s"status = $status",
        s"totalCapacity = $totalCapacity",
        s"usedCapacity = $usedCapacity",
        s"availableCapacity = $availableCapacity",
        s"readSpeed = $readSpeed",
        s"writeSpeed = $writeSpeed"
      ).mkString("StorageInformation(", ", ", ")")
  }

  /**
    * 262 CAMERA_CAPTURE_STATUS
    *
    * WIP: Information about the status of a capture
    *
    * @param timeBootMS        Timestamp (milliseconds since system boot)
    * @param imageStatus       Current status of image capturing (0: idle, 1: capture in progress, 2: interval set but idle, 3: interval set and capture in progress)
    * @param videoStatus       Current status of video capturing (0: idle, 1: capture in progress)
    * @param imageInterval     Image capture interval in seconds
    * @param recordingTimeMS   Time in milliseconds since recording started
    * @param availableCapacity Available storage capacity in MiB
    */
  case class CameraCaptureStatus(
    timeBootMS: Int,
    imageStatus: Int,
    videoStatus: Int,
    imageInterval: Float,
    recordingTimeMS: Int,
    availableCapacity: Float
  ) extends MAVMessage("CAMERA_CAPTURE_STATUS", systemId, componentId, protocolVersion) {
    set("time_boot_ms", timeBootMS)
    set("image_status", imageStatus)
    set("video_status", videoStatus)
    set("image_interval", imageInterval)
    set("recording_time_ms", recordingTimeMS)
    set("available_capacity", availableCapacity)

    override def toString: String =
      List(
        s"timeBootMS = $timeBootMS",
        s"imageStatus = $imageStatus",
        s"videoStatus = $videoStatus",
        s"imageInterval = $imageInterval",
        s"recordingTimeMS = $recordingTimeMS",
        s"availableCapacity = $availableCapacity"
      ).mkString("CameraCaptureStatus(", ", ", ")")
  }

  /**
    * 263 CAMERA_IMAGE_CAPTURED
    *
    * Information about a captured image
    *
    * @param timeBootMS    Timestamp (milliseconds since system boot)
    * @param timeUTC       Timestamp (microseconds since UNIX epoch) in UTC. 0 for unknown.
    * @param cameraId      Camera ID (1 for first, 2 for second, etc.)
    * @param lat           Latitude, expressed as degrees * 1E7 where image was taken
    * @param lon           Longitude, expressed as degrees * 1E7 where capture was taken
    * @param alt           Altitude in meters, expressed as * 1E3 (AMSL, not WGS84) where image was taken
    * @param relativeAlt   Altitude above ground in meters, expressed as * 1E3 where image was taken
    * @param q             Quaternion of camera orientation (w, x, y, z order, zero-rotation is 0, 0, 0, 0)
    * @param imageIndex    Zero based index of this image (image count since armed -1)
    * @param captureResult Boolean indicating success (1) or failure (0) while capturing this image.
    * @param fileURL       URL of image taken. Either local storage or http://foo.jpg if camera provides an HTTP interface.
    */
  case class CameraImageCaptured(
    timeBootMS: Int,
    timeUTC: Long,
    cameraId: Int,
    lat: Int,
    lon: Int,
    alt: Int,
    relativeAlt: Int,
    q: IndexedSeq[Float],
    imageIndex: Int,
    captureResult: Int,
    fileURL: String
  ) extends MAVMessage("CAMERA_IMAGE_CAPTURED", systemId, componentId, protocolVersion) {
    set("time_boot_ms", timeBootMS)
    set("time_utc", timeUTC)
    set("camera_id", cameraId)
    set("lat", lat)
    set("lon", lon)
    set("alt", alt)
    set("relative_alt", relativeAlt)
    set("q", q.map(float2Float).toArray)
    set("image_index", imageIndex)
    set("capture_result", captureResult)
    set("file_url", fileURL)

    override def toString: String =
      List(
        s"timeBootMS = $timeBootMS",
        s"timeUTC = $timeUTC",
        s"cameraId = $cameraId",
        s"lat = $lat",
        s"lon = $lon",
        s"alt = $alt",
        s"relativeAlt = $relativeAlt",
        s"q = $q",
        s"imageIndex = $imageIndex",
        s"captureResult = $captureResult",
        s"fileURL = $fileURL"
      ).mkString("CameraImageCaptured(", ", ", ")")
  }

  /**
    * 264 FLIGHT_INFORMATION
    *
    * WIP: Information about flight since last arming
    *
    * @param timeBootMS     Timestamp (milliseconds since system boot)
    * @param armingTimeUTC  Timestamp at arming (microseconds since UNIX epoch) in UTC, 0 for unknown
    * @param takeoffTimeUTC Timestamp at takeoff (microseconds since UNIX epoch) in UTC, 0 for unknown
    * @param flightUUID     Universally unique identifier (UUID) of flight, should correspond to name of logfiles
    */
  case class FlightInformation(
    timeBootMS: Int,
    armingTimeUTC: Long,
    takeoffTimeUTC: Long,
    flightUUID: Long
  ) extends MAVMessage("FLIGHT_INFORMATION", systemId, componentId, protocolVersion) {
    set("time_boot_ms", timeBootMS)
    set("arming_time_utc", armingTimeUTC)
    set("takeoff_time_utc", takeoffTimeUTC)
    set("flight_uuid", flightUUID)

    override def toString: String =
      List(
        s"timeBootMS = $timeBootMS",
        s"armingTimeUTC = $armingTimeUTC",
        s"takeoffTimeUTC = $takeoffTimeUTC",
        s"flightUUID = $flightUUID"
      ).mkString("FlightInformation(", ", ", ")")
  }

  /**
    * 265 MOUNT_ORIENTATION
    *
    * WIP: Orientation of a mount
    *
    * @param timeBootMS Timestamp (milliseconds since system boot)
    * @param roll       Roll in degrees
    * @param pitch      Pitch in degrees
    * @param yaw        Yaw in degrees
    */
  case class MountOrientation(
    timeBootMS: Int,
    roll: Float,
    pitch: Float,
    yaw: Float
  ) extends MAVMessage("MOUNT_ORIENTATION", systemId, componentId, protocolVersion) {
    set("time_boot_ms", timeBootMS)
    set("roll", roll)
    set("pitch", pitch)
    set("yaw", yaw)

    override def toString: String =
      List(
        s"timeBootMS = $timeBootMS",
        s"roll = $roll",
        s"pitch = $pitch",
        s"yaw = $yaw"
      ).mkString("MountOrientation(", ", ", ")")
  }

  /**
    * 266 LOGGING_DATA
    *
    * A message containing logged data (see also MAV_CMD_LOGGING_START)
    *
    * @param targetSystem       system ID of the target
    * @param targetComponent    component ID of the target
    * @param sequence           sequence number (can wrap)
    * @param length             data length
    * @param firstMessageOffset offset into data where first message starts. This can be used for recovery, when a previous message got lost (set to 255 if no start exists).
    * @param data               logged data
    */
  case class LoggingData(
    targetSystem: Int,
    targetComponent: Int,
    sequence: Int,
    length: Int,
    firstMessageOffset: Int,
    data: IndexedSeq[Int]
  ) extends MAVMessage("LOGGING_DATA", systemId, componentId, protocolVersion) {
    set("target_system", targetSystem)
    set("target_component", targetComponent)
    set("sequence", sequence)
    set("length", length)
    set("first_message_offset", firstMessageOffset)
    set("data", data.map(int2Integer).toArray)

    override def toString: String =
      List(
        s"targetSystem = $targetSystem",
        s"targetComponent = $targetComponent",
        s"sequence = $sequence",
        s"length = $length",
        s"firstMessageOffset = $firstMessageOffset",
        s"data = $data"
      ).mkString("LoggingData(", ", ", ")")
  }

  /**
    * 267 LOGGING_DATA_ACKED
    *
    * A message containing logged data which requires a LOGGING_ACK to be sent back
    *
    * @param targetSystem       system ID of the target
    * @param targetComponent    component ID of the target
    * @param sequence           sequence number (can wrap)
    * @param length             data length
    * @param firstMessageOffset offset into data where first message starts. This can be used for recovery, when a previous message got lost (set to 255 if no start exists).
    * @param data               logged data
    */
  case class LoggingDataAcked(
    targetSystem: Int,
    targetComponent: Int,
    sequence: Int,
    length: Int,
    firstMessageOffset: Int,
    data: IndexedSeq[Int]
  ) extends MAVMessage("LOGGING_DATA_ACKED", systemId, componentId, protocolVersion) {
    set("target_system", targetSystem)
    set("target_component", targetComponent)
    set("sequence", sequence)
    set("length", length)
    set("first_message_offset", firstMessageOffset)
    set("data", data.map(int2Integer).toArray)

    override def toString: String =
      List(
        s"targetSystem = $targetSystem",
        s"targetComponent = $targetComponent",
        s"sequence = $sequence",
        s"length = $length",
        s"firstMessageOffset = $firstMessageOffset",
        s"data = $data"
      ).mkString("LoggingDataAcked(", ", ", ")")
  }

  /**
    * 268 LOGGING_ACK
    *
    * An ack for a LOGGING_DATA_ACKED message
    *
    * @param targetSystem    system ID of the target
    * @param targetComponent component ID of the target
    * @param sequence        sequence number (must match the one in LOGGING_DATA_ACKED)
    */
  case class LoggingAck(
    targetSystem: Int,
    targetComponent: Int,
    sequence: Int
  ) extends MAVMessage("LOGGING_ACK", systemId, componentId, protocolVersion) {
    set("target_system", targetSystem)
    set("target_component", targetComponent)
    set("sequence", sequence)

    override def toString: String =
      List(
        s"targetSystem = $targetSystem",
        s"targetComponent = $targetComponent",
        s"sequence = $sequence"
      ).mkString("LoggingAck(", ", ", ")")
  }

  /**
    * 269 VIDEO_STREAM_INFORMATION
    *
    * WIP: Information about video stream
    *
    * @param cameraId    Camera ID (1 for first, 2 for second, etc.)
    * @param status      Current status of video streaming (0: not running, 1: in progress)
    * @param frameRate   Frames per second
    * @param resolutionH Resolution horizontal in pixels
    * @param resolutionV Resolution vertical in pixels
    * @param bitRate     Bit rate in bits per second
    * @param rotation    Video image rotation clockwise
    * @param uRI         Video stream URI
    */
  case class VideoStreamInformation(
    cameraId: Int,
    status: Int,
    frameRate: Float,
    resolutionH: Int,
    resolutionV: Int,
    bitRate: Int,
    rotation: Int,
    uRI: String
  ) extends MAVMessage("VIDEO_STREAM_INFORMATION", systemId, componentId, protocolVersion) {
    set("camera_id", cameraId)
    set("status", status)
    set("framerate", frameRate)
    set("resolution_h", resolutionH)
    set("resolution_v", resolutionV)
    set("bitrate", bitRate)
    set("rotation", rotation)
    set("uri", uRI)

    override def toString: String =
      List(
        s"cameraId = $cameraId",
        s"status = $status",
        s"frameRate = $frameRate",
        s"resolutionH = $resolutionH",
        s"resolutionV = $resolutionV",
        s"bitRate = $bitRate",
        s"rotation = $rotation",
        s"uRI = $uRI"
      ).mkString("VideoStreamInformation(", ", ", ")")
  }

  /**
    * 270 SET_VIDEO_STREAM_SETTINGS
    *
    * WIP: Message that sets video stream settings
    *
    * @param targetSystem    system ID of the target
    * @param targetComponent component ID of the target
    * @param cameraId        Camera ID (1 for first, 2 for second, etc.)
    * @param frameRate       Frames per second (set to -1 for highest framerate possible)
    * @param resolutionH     Resolution horizontal in pixels (set to -1 for highest resolution possible)
    * @param resolutionV     Resolution vertical in pixels (set to -1 for highest resolution possible)
    * @param bitRate         Bit rate in bits per second (set to -1 for auto)
    * @param rotation        Video image rotation clockwise (0-359 degrees)
    * @param uRI             Video stream URI
    */
  case class SetVideoStreamSettings(
    targetSystem: Int,
    targetComponent: Int,
    cameraId: Int,
    frameRate: Float,
    resolutionH: Int,
    resolutionV: Int,
    bitRate: Int,
    rotation: Int,
    uRI: String
  ) extends MAVMessage("SET_VIDEO_STREAM_SETTINGS", systemId, componentId, protocolVersion) {
    set("target_system", targetSystem)
    set("target_component", targetComponent)
    set("camera_id", cameraId)
    set("framerate", frameRate)
    set("resolution_h", resolutionH)
    set("resolution_v", resolutionV)
    set("bitrate", bitRate)
    set("rotation", rotation)
    set("uri", uRI)

    override def toString: String =
      List(
        s"targetSystem = $targetSystem",
        s"targetComponent = $targetComponent",
        s"cameraId = $cameraId",
        s"frameRate = $frameRate",
        s"resolutionH = $resolutionH",
        s"resolutionV = $resolutionV",
        s"bitRate = $bitRate",
        s"rotation = $rotation",
        s"uRI = $uRI"
      ).mkString("SetVideoStreamSettings(", ", ", ")")
  }

  /**
    * 299 WIFI_CONFIG_AP
    *
    * Configure AP SSID and Password.
    *
    * @param ssid     Name of Wi-Fi network (SSID). Leave it blank to leave it unchanged.
    * @param password Password. Leave it blank for an open AP.
    */
  case class WifiConfigAp(
    ssid: String,
    password: String
  ) extends MAVMessage("WIFI_CONFIG_AP", systemId, componentId, protocolVersion) {
    set("ssid", ssid)
    set("password", password)

    override def toString: String =
      List(
        s"ssid = $ssid",
        s"password = $password"
      ).mkString("WifiConfigAp(", ", ", ")")
  }

  /**
    * 300 PROTOCOL_VERSION
    *
    * WIP: Version and capability of protocol version. This message is the response to REQUEST_PROTOCOL_VERSION and is used as part of the handshaking to establish which MAVLink version should be used on the network. Every node should respond to REQUEST_PROTOCOL_VERSION to enable the handshaking. Library implementers should consider adding this into the default decoding state machine to allow the protocol core to respond directly.
    *
    * @param version            Currently active MAVLink version number * 100: v1.0 is 100, v2.0 is 200, etc.
    * @param minVersion         Minimum MAVLink version supported
    * @param maxVersion         Maximum MAVLink version supported (set to the same value as version by default)
    * @param specVersionHash    The first 8 bytes (not characters printed in hex!) of the git hash.
    * @param libraryVersionHash The first 8 bytes (not characters printed in hex!) of the git hash.
    */
  case class ProtocolVersion(
    version: Int,
    minVersion: Int,
    maxVersion: Int,
    specVersionHash: IndexedSeq[Int],
    libraryVersionHash: IndexedSeq[Int]
  ) extends MAVMessage("PROTOCOL_VERSION", systemId, componentId, protocolVersion) {
    set("version", version)
    set("min_version", minVersion)
    set("max_version", maxVersion)
    set("spec_version_hash", specVersionHash.map(int2Integer).toArray)
    set("library_version_hash", libraryVersionHash.map(int2Integer).toArray)

    override def toString: String =
      List(
        s"version = $version",
        s"minVersion = $minVersion",
        s"maxVersion = $maxVersion",
        s"specVersionHash = $specVersionHash",
        s"libraryVersionHash = $libraryVersionHash"
      ).mkString("ProtocolVersion(", ", ", ")")
  }

  /**
    * 310 UAVCAN_NODE_STATUS
    *
    * General status information of an UAVCAN node. Please refer to the definition of the UAVCAN message "uavcan.protocol.NodeStatus" for the background information. The UAVCAN specification is available at http://uavcan.org.
    *
    * @param timeUSec                 Timestamp (microseconds since UNIX epoch or microseconds since system boot)
    * @param uptimeSec                The number of seconds since the start-up of the node.
    * @param health                   Generalized node health status.
    * @param mode                     Generalized operating mode.
    * @param subMode                  Not used currently.
    * @param vendorSpecificStatusCode Vendor-specific status information.
    */
  case class UAVCANNodeStatus(
    timeUSec: Long,
    uptimeSec: Int,
    health: UAVCANNodeHealth,
    mode: UAVCANNodeMode,
    subMode: Int,
    vendorSpecificStatusCode: Int
  ) extends MAVMessage("UAVCAN_NODE_STATUS", systemId, componentId, protocolVersion) {
    set("time_usec", timeUSec)
    set("uptime_sec", uptimeSec)
    set("health", health.value)
    set("mode", mode.value)
    set("sub_mode", subMode)
    set("vendor_specific_status_code", vendorSpecificStatusCode)

    override def toString: String =
      List(
        s"timeUSec = $timeUSec",
        s"uptimeSec = $uptimeSec",
        s"health = $health",
        s"mode = $mode",
        s"subMode = $subMode",
        s"vendorSpecificStatusCode = $vendorSpecificStatusCode"
      ).mkString("UAVCANNodeStatus(", ", ", ")")
  }

  /**
    * 311 UAVCAN_NODE_INFO
    *
    * General information describing a particular UAVCAN node. Please refer to the definition of the UAVCAN service "uavcan.protocol.GetNodeInfo" for the background information. This message should be emitted by the system whenever a new node appears online, or an existing node reboots. Additionally, it can be emitted upon request from the other end of the MAVLink channel (see MAV_CMD_UAVCAN_GET_NODE_INFO). It is also not prohibited to emit this message unconditionally at a low frequency. The UAVCAN specification is available at http://uavcan.org.
    *
    * @param timeUSec       Timestamp (microseconds since UNIX epoch or microseconds since system boot)
    * @param uptimeSec      The number of seconds since the start-up of the node.
    * @param name           Node name string. For example, "sapog.px4.io".
    * @param hWVersionMajor Hardware major version number.
    * @param hWVersionMinor Hardware minor version number.
    * @param hWUniqueId     Hardware unique 128-bit ID.
    * @param sWVersionMajor Software major version number.
    * @param sWVersionMinor Software minor version number.
    * @param sWVcsCommit    Version control system (VCS) revision identifier (e.g. git short commit hash). Zero if unknown.
    */
  case class UAVCANNodeInfo(
    timeUSec: Long,
    uptimeSec: Int,
    name: String,
    hWVersionMajor: Int,
    hWVersionMinor: Int,
    hWUniqueId: IndexedSeq[Int],
    sWVersionMajor: Int,
    sWVersionMinor: Int,
    sWVcsCommit: Int
  ) extends MAVMessage("UAVCAN_NODE_INFO", systemId, componentId, protocolVersion) {
    set("time_usec", timeUSec)
    set("uptime_sec", uptimeSec)
    set("name", name)
    set("hw_version_major", hWVersionMajor)
    set("hw_version_minor", hWVersionMinor)
    set("hw_unique_id", hWUniqueId.map(int2Integer).toArray)
    set("sw_version_major", sWVersionMajor)
    set("sw_version_minor", sWVersionMinor)
    set("sw_vcs_commit", sWVcsCommit)

    override def toString: String =
      List(
        s"timeUSec = $timeUSec",
        s"uptimeSec = $uptimeSec",
        s"name = $name",
        s"hWVersionMajor = $hWVersionMajor",
        s"hWVersionMinor = $hWVersionMinor",
        s"hWUniqueId = $hWUniqueId",
        s"sWVersionMajor = $sWVersionMajor",
        s"sWVersionMinor = $sWVersionMinor",
        s"sWVcsCommit = $sWVcsCommit"
      ).mkString("UAVCANNodeInfo(", ", ", ")")
  }

  /**
    * 320 PARAM_EXT_REQUEST_READ
    *
    * Request to read the value of a parameter with the either the param_id string id or param_index.
    *
    * @param targetSystem    System ID
    * @param targetComponent Component ID
    * @param paramId         Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
    * @param paramIndex      Parameter index. Set to -1 to use the Parameter ID field as identifier (else param_id will be ignored)
    */
  case class ParamExtRequestRead(
    targetSystem: Int,
    targetComponent: Int,
    paramId: String,
    paramIndex: Int
  ) extends MAVMessage("PARAM_EXT_REQUEST_READ", systemId, componentId, protocolVersion) {
    set("target_system", targetSystem)
    set("target_component", targetComponent)
    set("param_id", paramId)
    set("param_index", paramIndex)

    override def toString: String =
      List(
        s"targetSystem = $targetSystem",
        s"targetComponent = $targetComponent",
        s"paramId = $paramId",
        s"paramIndex = $paramIndex"
      ).mkString("ParamExtRequestRead(", ", ", ")")
  }

  /**
    * 321 PARAM_EXT_REQUEST_LIST
    *
    * Request all parameters of this component. After this request, all parameters are emitted.
    *
    * @param targetSystem    System ID
    * @param targetComponent Component ID
    */
  case class ParamExtRequestList(
    targetSystem: Int,
    targetComponent: Int
  ) extends MAVMessage("PARAM_EXT_REQUEST_LIST", systemId, componentId, protocolVersion) {
    set("target_system", targetSystem)
    set("target_component", targetComponent)

    override def toString: String =
      List(
        s"targetSystem = $targetSystem",
        s"targetComponent = $targetComponent"
      ).mkString("ParamExtRequestList(", ", ", ")")
  }

  /**
    * 322 PARAM_EXT_VALUE
    *
    * Emit the value of a parameter. The inclusion of param_count and param_index in the message allows the recipient to keep track of received parameters and allows them to re-request missing parameters after a loss or timeout.
    *
    * @param paramId    Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
    * @param paramValue Parameter value
    * @param paramType  Parameter type: see the MAV_PARAM_EXT_TYPE enum for supported data types.
    * @param paramCount Total number of parameters
    * @param paramIndex Index of this parameter
    */
  case class ParamExtValue(
    paramId: String,
    paramValue: String,
    paramType: MAVParamExtType,
    paramCount: Int,
    paramIndex: Int
  ) extends MAVMessage("PARAM_EXT_VALUE", systemId, componentId, protocolVersion) {
    set("param_id", paramId)
    set("param_value", paramValue)
    set("param_type", paramType.value)
    set("param_count", paramCount)
    set("param_index", paramIndex)

    override def toString: String =
      List(
        s"paramId = $paramId",
        s"paramValue = $paramValue",
        s"paramType = $paramType",
        s"paramCount = $paramCount",
        s"paramIndex = $paramIndex"
      ).mkString("ParamExtValue(", ", ", ")")
  }

  /**
    * 323 PARAM_EXT_SET
    *
    * Set a parameter value. In order to deal with message loss (and retransmission of PARAM_EXT_SET), when setting a parameter value and the new value is the same as the current value, you will immediately get a PARAM_ACK_ACCEPTED response. If the current state is PARAM_ACK_IN_PROGRESS, you will accordingly receive a PARAM_ACK_IN_PROGRESS in response.
    *
    * @param targetSystem    System ID
    * @param targetComponent Component ID
    * @param paramId         Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
    * @param paramValue      Parameter value
    * @param paramType       Parameter type: see the MAV_PARAM_EXT_TYPE enum for supported data types.
    */
  case class ParamExtSet(
    targetSystem: Int,
    targetComponent: Int,
    paramId: String,
    paramValue: String,
    paramType: MAVParamExtType
  ) extends MAVMessage("PARAM_EXT_SET", systemId, componentId, protocolVersion) {
    set("target_system", targetSystem)
    set("target_component", targetComponent)
    set("param_id", paramId)
    set("param_value", paramValue)
    set("param_type", paramType.value)

    override def toString: String =
      List(
        s"targetSystem = $targetSystem",
        s"targetComponent = $targetComponent",
        s"paramId = $paramId",
        s"paramValue = $paramValue",
        s"paramType = $paramType"
      ).mkString("ParamExtSet(", ", ", ")")
  }

  /**
    * 324 PARAM_EXT_ACK
    *
    * Response from a PARAM_EXT_SET message.
    *
    * @param paramId     Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
    * @param paramValue  Parameter value (new value if PARAM_ACK_ACCEPTED, current value otherwise)
    * @param paramType   Parameter type: see the MAV_PARAM_EXT_TYPE enum for supported data types.
    * @param paramResult Result code: see the PARAM_ACK enum for possible codes.
    */
  case class ParamExtAck(
    paramId: String,
    paramValue: String,
    paramType: MAVParamExtType,
    paramResult: ParamAck
  ) extends MAVMessage("PARAM_EXT_ACK", systemId, componentId, protocolVersion) {
    set("param_id", paramId)
    set("param_value", paramValue)
    set("param_type", paramType.value)
    set("param_result", paramResult.value)

    override def toString: String =
      List(
        s"paramId = $paramId",
        s"paramValue = $paramValue",
        s"paramType = $paramType",
        s"paramResult = $paramResult"
      ).mkString("ParamExtAck(", ", ", ")")
  }

  /**
    * 330 OBSTACLE_DISTANCE
    *
    * Obstacle distances in front of the sensor, starting from the left in increment degrees to the right
    *
    * @param timeUSec    Timestamp (microseconds since system boot or since UNIX epoch)
    * @param sensorType  Class id of the distance sensor type.
    * @param distances   Distance of obstacles in front of the sensor starting on the left side. A value of 0 means that the obstacle is right in front of the sensor. A value of max_distance +1 means no obstace is present. A value of UINT16_MAX for unknown/not used. In a array element, each unit corresponds to 1cm.
    * @param increment   Angular width in degrees of each array element.
    * @param minDistance Minimum distance the sensor can measure in centimeters
    * @param maxDistance Maximum distance the sensor can measure in centimeters
    */
  case class ObstacleDistance(
    timeUSec: Long,
    sensorType: MAVDistanceSensor,
    distances: IndexedSeq[Int],
    increment: Int,
    minDistance: Int,
    maxDistance: Int
  ) extends MAVMessage("OBSTACLE_DISTANCE", systemId, componentId, protocolVersion) {
    set("time_usec", timeUSec)
    set("sensor_type", sensorType.value)
    set("distances", distances.map(int2Integer).toArray)
    set("increment", increment)
    set("min_distance", minDistance)
    set("max_distance", maxDistance)

    override def toString: String =
      List(
        s"timeUSec = $timeUSec",
        s"sensorType = $sensorType",
        s"distances = $distances",
        s"increment = $increment",
        s"minDistance = $minDistance",
        s"maxDistance = $maxDistance"
      ).mkString("ObstacleDistance(", ", ", ")")
  }

}
