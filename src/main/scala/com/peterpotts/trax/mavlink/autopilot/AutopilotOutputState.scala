package com.peterpotts.trax.mavlink.autopilot

import com.peterpotts.trax.mavlink.model.Autopilot

/**
  * @author Peter Potts
  */
object AutopilotOutputState {

  /**
    * {{{
    * onboardControlSensorsPresent Bitmask showing which
    *                              onboard controllers and sensors are present.
    *                              Value of 0: not present.
    *                              Value of 1: present.
    *                              Indices defined by ENUM MAV_SYS_STATUS_SENSOR
    * onboardControlSensorsEnabled Bitmask showing which
    *                              onboard controllers and sensors are enabled:
    *                              Value of 0: not enabled.
    *                              Value of 1: enabled.
    *                              Indices defined by ENUM MAV_SYS_STATUS_SENSOR
    * onboardControlSensorsHealth  Bitmask showing which
    *                              onboard controllers and sensors are
    *                              operational or have an error:
    *                              Value of 0: not enabled.
    *                              Value of 1: enabled.
    *                              Indices defined by ENUM MAV_SYS_STATUS_SENSOR
    * load                         Maximum usage in percent of the main loop time,
    *                              (0%: 0, 100%: 1000) should be always below 1000
    * voltageBattery               Battery voltage, in millivolts (1 = 1 millivolt)
    * currentBattery               Battery current, in 10 * milliamperes
    *                              (1 = 10 milliampere),
    *                              -1: autopilot does not measure the current
    * batteryRemaining             Remaining battery energy: (0%: 0, 100%: 100),
    *                              -1: autopilot estimate the remaining battery
    * dropRateComm                 Communication drops in percent,
    *                              (0%: 0, 100%: 10'000),
    *                              (UART, I2C, SPI, CAN),
    *                              dropped packets on all links
    *                              (packets that were corrupted on reception
    *                              on the MAV)
    * errorsComm                   Communication errors (UART, I2C, SPI, CAN),
    *                              dropped packets on all links
    *                              (packets that were corrupted on reception
    *                              on the MAV)
    * errorsCount1                 Autopilot-specific errors
    * errorsCount2                 Autopilot-specific errors
    * errorsCount3                 Autopilot-specific errors
    * errorsCount4                 Autopilot-specific errors
    * }}}
    */
  object VarSysStatus extends VarMessage[Autopilot.SysStatus](1)

  /**
    * {{{
    * timeUSec          Timestamp (micros since boot or Unix epoch)
    * altitudeMonotonic This altitude measure is initialized on system boot and
    *                   monotonic (it is never reset,
    *                   but represents the local altitude change).
    *                   The only guarantee on this field is that
    *                   it will never be reset and
    *                   is consistent within a flight.
    *                   The recommended value for this field is
    *                   the uncorrected barometric altitude at boot time.
    *                   This altitude will also drift and vary between flights.
    * altitudeAMSL      This altitude measure is strictly above mean sea level
    *                   and might be non-monotonic
    *                   (it might reset on events like GPS lock or
    *                   when a new QNH value is set).
    *                   It should be the altitude to which
    *                   global altitude waypoints are compared to.
    *                   Note that it is *not* the GPS altitude,
    *                   however, most GPS modules
    *                   already output AMSL by default and
    *                   not the WGS84 altitude.
    * altitudeLocal     This is the local altitude in the local coordinate frame.
    *                   It is not the altitude above home,
    *                   but in reference to the coordinate origin (0, 0, 0).
    *                   It is up-positive.
    * altitudeRelative  This is the altitude above the home position.
    *                   It resets on each change of the current home position.
    * altitudeTerrain   This is the altitude above terrain.
    *                   It might be fed by a terrain database or an altimeter.
    *                   Values smaller than -1000 should be interpreted as unknown.
    * bottomClearance   This is not the altitude,
    *                   but the clear space below the system
    *                   according to the fused clearance estimate.
    *                   It generally should max out at the maximum range
    *                   of e.g. the laser altimeter.
    *                   It is generally a moving target.
    *                   A negative value indicates no measurement available.
    * }}}
    */
  object VarAltitude extends VarMessage[Autopilot.Altitude](1)

  /**
    * {{{
    * timeBootMS Timestamp (milliseconds since system boot)
    * q1         Quaternion component 1, w (1 in null-rotation)
    * q2         Quaternion component 2, x (0 in null-rotation)
    * q3         Quaternion component 3, y (0 in null-rotation)
    * q4         Quaternion component 4, z (0 in null-rotation)
    * rollSpeed  Roll angular speed (rad/s)
    * pitchSpeed Pitch angular speed (rad/s)
    * yawSpeed   Yaw angular speed (rad/s)
    * }}}
    */
  object VarAttitudeQuaternion extends VarMessage[Autopilot.AttitudeQuaternion](1)

  /**
    * {{{
    * timeBootMS  Timestamp (milliseconds since system boot)
    * lat         Latitude, expressed as degrees * 1E7
    * lon         Longitude, expressed as degrees * 1E7
    * alt         Altitude in meters, expressed as * 1000 (millimeters),
    *             AMSL (not WGS84 - note that virtually all
    *             GPS modules provide the AMSL as well)
    * relativeAlt Altitude above ground in meters, expressed as * 1000 (millimeters)
    * vX          Ground X Speed (Latitude, positive north), expressed as m/s * 100
    * vY          Ground Y Speed (Longitude, positive east), expressed as m/s * 100
    * vZ          Ground Z Speed (Altitude, positive down), expressed as m/s * 100
    * hdg         Vehicle heading (yaw angle) in degrees * 100, 0.0..359.99 degrees.
    *             If unknown, set to: UINT16_MAX
    * }}}
    */
  object VarGlobalPositionInt extends VarMessage[Autopilot.GlobalPositionInt](1)

  /**
    * {{{
    * timeUSec          Timestamp (microseconds since UNIX epoch or
    *                   microseconds since system boot)
    * fixType           See the GPS_FIX_TYPE enum.
    * lat               Latitude (WGS84, EGM96 ellipsoid), in degrees * 1E7
    * lon               Longitude (WGS84, EGM96 ellipsoid), in degrees * 1E7
    * alt               Altitude (AMSL, NOT WGS84),
    *                   in meters * 1000 (positive for up).
    *                   Note that virtually all GPS modules provide
    *                   the AMSL altitude in addition to the WGS84 altitude.
    * epH               GPS HDOP horizontal dilution of position (unitless).
    *                   If unknown, set to: UINT16_MAX
    * epV               GPS VDOP vertical dilution of position (unitless).
    *                   If unknown, set to: UINT16_MAX
    * vel               GPS ground speed (m/s * 100).
    *                   If unknown, set to: UINT16_MAX
    * cOG               Course over ground (NOT heading,
    *                   but direction of movement) in degrees * 100,
    *                   0.0..359.99 degrees.
    *                   If unknown, set to: UINT16_MAX
    * satellitesVisible Number of satellites visible.
    *                   If unknown, set to 255
    * altEllipsoid      Altitude (above WGS84, EGM96 ellipsoid),
    *                   in meters * 1000 (positive for up).
    * hAcc              Position uncertainty in meters * 1000
    *                   (positive for up).
    * vAcc              Altitude uncertainty in meters * 1000
    *                   (positive for up).
    * velAcc            Speed uncertainty in meters * 1000
    *                   (positive for up).
    * hdgAcc            Heading / track uncertainty in degrees * 1e5.
    * }}}
    */
  object VarGPSRawInt extends VarMessage[Autopilot.GPSRawInt](1)

  /**
    * {{{
    * timeBootMS Timestamp (milliseconds since system boot)
    * x          X Position
    * y          Y Position
    * z          Z Position
    * vX         X Speed
    * vY         Y Speed
    * vZ         Z Speed
    * }}}
    */
  object VarLocalPositionNED extends VarMessage[Autopilot.LocalPositionNED](1)

  /**
    * {{{
    * airspeed    Current airspeed in m/s
    * groundSpeed Current ground speed in m/s
    * heading     Current heading in degrees, in compass units (0..360, 0=north)
    * throttle    Current throttle setting in integer percent, 0 to 100
    * alt         Current altitude (MSL), in meters
    * climb       Current climb rate in meters/second
    * }}}
    */
  object VarVFRHUD extends VarMessage[Autopilot.VFRHUD](1)

  /**
    * {{{
    * timeBootMS Timestamp (milliseconds since system boot)
    * roll       Roll angle (rad, -pi..+pi)
    * pitch      Pitch angle (rad, -pi..+pi)
    * yaw        Yaw angle (rad, -pi..+pi)
    * rollSpeed  Roll angular speed (rad/s)
    * pitchSpeed Pitch angular speed (rad/s)
    * yawSpeed   Yaw angular speed (rad/s)
    * }}}
    */
  object VarAttitude extends VarMessage[Autopilot.Attitude](1)

  /**
    * {{{
    * vTOLState   The VTOL state if applicable.
    *             Is set to MAV_VTOL_STATE_UNDEFINED
    *             if UAV is not in VTOL configuration.
    * landedState The landed state.
    *             Is set to MAV_LANDED_STATE_UNDEFINED
    *             if landed state is unknown.
    * }}}
    */
  object VarExtendedSysState extends VarMessage[Autopilot.ExtendedSysState](1)

  /**
    * {{{
    * timeBootMS Timestamp (milliseconds since system boot)
    * chanCount  Total number of RC channels being received.
    *            This can be larger than 18,
    *            indicating that more channels are available
    *            but not given in this message.
    *            This value should be 0 when no RC channels are available.
    * chan1Raw   RC channel 1 value, in microseconds.
    *            A value of UINT16_MAX implies the channel is unused.
    * chan2Raw   RC channel 2 value, in microseconds.
    *            A value of UINT16_MAX implies the channel is unused.
    * chan3Raw   RC channel 3 value, in microseconds.
    *            A value of UINT16_MAX implies the channel is unused.
    * chan4Raw   RC channel 4 value, in microseconds.
    *            A value of UINT16_MAX implies the channel is unused.
    * chan5Raw   RC channel 5 value, in microseconds.
    *            A value of UINT16_MAX implies the channel is unused.
    * chan6Raw   RC channel 6 value, in microseconds.
    *            A value of UINT16_MAX implies the channel is unused.
    * chan7Raw   RC channel 7 value, in microseconds.
    *            A value of UINT16_MAX implies the channel is unused.
    * chan8Raw   RC channel 8 value, in microseconds.
    *            A value of UINT16_MAX implies the channel is unused.
    * chan9Raw   RC channel 9 value, in microseconds.
    *            A value of UINT16_MAX implies the channel is unused.
    * chan10Raw  RC channel 10 value, in microseconds.
    *            A value of UINT16_MAX implies the channel is unused.
    * chan11Raw  RC channel 11 value, in microseconds.
    *            A value of UINT16_MAX implies the channel is unused.
    * chan12Raw  RC channel 12 value, in microseconds.
    *            A value of UINT16_MAX implies the channel is unused.
    * chan13Raw  RC channel 13 value, in microseconds.
    *            A value of UINT16_MAX implies the channel is unused.
    * chan14Raw  RC channel 14 value, in microseconds.
    *            A value of UINT16_MAX implies the channel is unused.
    * chan15Raw  RC channel 15 value, in microseconds.
    *            A value of UINT16_MAX implies the channel is unused.
    * chan16Raw  RC channel 16 value, in microseconds.
    *            A value of UINT16_MAX implies the channel is unused.
    * chan17Raw  RC channel 17 value, in microseconds.
    *            A value of UINT16_MAX implies the channel is unused.
    * chan18Raw  RC channel 18 value, in microseconds.
    *            A value of UINT16_MAX implies the channel is unused.
    * rSSI       Receive signal strength indicator,
    *            0: 0%, 100: 100%, 255: invalid/unknown.
    * }}}
    */
  object VarRCChannels extends VarMessage[Autopilot.RCChannels](1)

  /**
    * {{{
    * timeUSec   Timestamp (microseconds since system boot)
    * port       Servo output port (set of 8 outputs = 1 port).
    *            Most MAVs will just use one,
    *            but this allows to encode more than 8 servos.
    * servo1Raw  Servo output 1 value, in microseconds
    * servo2Raw  Servo output 2 value, in microseconds
    * servo3Raw  Servo output 3 value, in microseconds
    * servo4Raw  Servo output 4 value, in microseconds
    * servo5Raw  Servo output 5 value, in microseconds
    * servo6Raw  Servo output 6 value, in microseconds
    * servo7Raw  Servo output 7 value, in microseconds
    * servo8Raw  Servo output 8 value, in microseconds
    * servo9Raw  Servo output 9 value, in microseconds
    * servo10Raw Servo output 10 value, in microseconds
    * servo11Raw Servo output 11 value, in microseconds
    * servo12Raw Servo output 12 value, in microseconds
    * servo13Raw Servo output 13 value, in microseconds
    * servo14Raw Servo output 14 value, in microseconds
    * servo15Raw Servo output 15 value, in microseconds
    * servo16Raw Servo output 16 value, in microseconds
    * }}}
    */
  object VarServoOutputRaw extends VarMessage[Autopilot.ServoOutputRaw](1)

  /**
    * {{{
    * timeUSec Timestamp (microseconds since UNIX epoch or
    *          microseconds since system boot)
    * controls Control outputs -1 .. 1.
    *          Channel assignment depends on the simulated hardware.
    * mode     System mode (MAV_MODE), includes arming state.
    * flags    Flags as bitfield, reserved for future use.
    * }}}
    */
  object VarHILActuatorControls extends VarMessage[Autopilot.HILActuatorControls](1)
}
