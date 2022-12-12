package com.peterpotts.trax.mavlink.model

/**
  * @author Peter Potts
  */
object MAVFormatter {
  private val specials =
    Map(
      "adsb" -> "ADSB",
      "afx" -> "AFX",
      "afy" -> "AFY",
      "afz" -> "AFZ",
      "ahrs" -> "AHRS",
      "amsl" -> "AMSL",
      "ardupilotmega" -> "ArduPilotMega",
      "asluav" -> "ASLUAV",
      "aspd" -> "ASPD",
      "autocontinue" -> "AutoContinue",
      "autoquad" -> "AutoQuad",
      "ax" -> "AX",
      "ay" -> "AY",
      "az" -> "AZ",
      "baudrate" -> "BaudRate",
      "bitrate" -> "BitRate",
      "bmp" -> "BMP",
      "chancount" -> "ChanCount",
      "cog" -> "COG",
      "dgps" -> "DGPS",
      "ekf" -> "EKF",
      "enu" -> "ENU",
      "eph" -> "EpH",
      "epv" -> "EpV",
      "fp" -> "FP",
      "framerate" -> "FrameRate",
      "ftp" -> "FTP",
      "fw" -> "FW",
      "gcs" -> "GCS",
      "gps" -> "GPS",
      "gps2" -> "GPS2",
      "gridbit" -> "GridBit",
      "groundspeed" -> "GroundSpeed",
      "hagl" -> "HAGL",
      "hdop" -> "HDOP",
      "hil" -> "HIL",
      "hud" -> "HUD",
      "hw" -> "HW",
      "iar" -> "IAR",
      "imu" -> "IMU",
      "imu2" -> "IMU2",
      "imu3" -> "IMU3",
      "jpeg" -> "JPEG",
      "jpg" -> "JPG",
      "life" -> "LiFe",
      "lipo" -> "LiPo",
      "mav" -> "MAV",
      "mavlink" -> "MAVLink",
      "mavlink2" -> "MAVLink2",
      "maxalt" -> "MaxAlt",
      "mc" -> "MC",
      "minalt" -> "MinAlt",
      "missionplanner" -> "MissionPlanner",
      "mlx" -> "MLX",
      "mm" -> "MM",
      "ms" -> "MS",
      "ned" -> "NED",
      "nimh" -> "NiMH",
      "nsats" -> "NSats",
      "numch" -> "NumCh",
      "ok" -> "OK",
      "openpilot" -> "OpenPilot",
      "os" -> "OS",
      "osd" -> "OSD",
      "pathplanner" -> "PathPlanner",
      "p1x" -> "P1X",
      "p1y" -> "P1Y",
      "p1z" -> "P1Z",
      "p2x" -> "P2X",
      "p2y" -> "P2Y",
      "p2z" -> "P2Z",
      "pgm" -> "PGM",
      "pitchspeed" -> "PitchSpeed",
      "png" -> "PNG",
      "ppp" -> "PPP",
      "ppz" -> "PPZ",
      "prn" -> "PRN",
      "pwm" -> "PWM",
      "px4" -> "PX4",
      "qnh" -> "QNH",
      "qx1" -> "QX1",
      "rc" -> "RC",
      "remnoise" -> "RemNoise",
      "remrssi" -> "RemRSSI",
      "roi" -> "ROI",
      "rollspeed" -> "RollSpeed",
      "rssi" -> "RSSI",
      "rtcm" -> "RTCM",
      "rtk" -> "RTK",
      "rtl" -> "RTL",
      "rx" -> "RX",
      "rxerrors" -> "RXErrors",
      "seqnr" -> "SeqNr",
      "smaccmpilot" -> "SMACCMPilot",
      "smartap" -> "SmartAP",
      "snr" -> "SNR",
      "sw" -> "SW",
      "tas" -> "TAS",
      "tow" -> "TOW",
      "tx" -> "TX",
      "txbuf" -> "TXBuf",
      "uart" -> "UART",
      "uav" -> "UAV",
      "uavcan" -> "UAVCAN",
      "udb" -> "UDB",
      "udp" -> "UDP",
      "uri" -> "URI",
      "url" -> "URL",
      "us" -> "US",
      "usb" -> "USB",
      "usec" -> "USec",
      "utc" -> "UTC",
      "uuid" -> "UUID",
      "vd" -> "VD",
      "vdop" -> "VDOP",
      "ve" -> "VE",
      "vio" -> "VIO",
      "vn" -> "VN",
      "vservo" -> "VServo",
      "vfr" -> "VFR",
      "vtol" -> "VTOL",
      "vx" -> "VX",
      "vy" -> "VY",
      "vz" -> "VZ",
      "wn" -> "WN",
      "wpindex" -> "WPIndex",
      "wpnext" -> "WPNext",
      "xacc" -> "XAcc",
      "xgyro" -> "XGyro",
      "xmag" -> "XMag",
      "xtrack" -> "XTrack",
      "xy" -> "XY",
      "yacc" -> "YAcc",
      "yawspeed" -> "YawSpeed",
      "ygyro" -> "YGyro",
      "ymag" -> "YMag",
      "zacc" -> "ZAcc",
      "zgyro" -> "ZGyro",
      "zmag" -> "ZMag"
    )

  def toClassName(name: String): String = {
    val value = name.split("_").map(up).mkString
    if (value.head.isDigit) "_" + value else value
  }

  def toVariableName(name: String): String =
    if (name == "type") {
      "`type`"
    } else {
      val head :: tail = name.split("_").toList
      val value = down(head) + tail.map(up).mkString
      if (value.head.isDigit) "_" + value else value
    }

  private def up(word: String): String = {
    val value = specials.getOrElse(word.toLowerCase, word.toLowerCase)
    value.updated(0, value.charAt(0).toUpper)
  }

  private def down(word: String): String = {
    val value = specials.getOrElse(word.toLowerCase, word.toLowerCase)
    value.updated(0, value.charAt(0).toLower)
  }
}
