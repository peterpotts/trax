package com.peterpotts.trax.mavlink.autopilot

import com.peterpotts.trax.mavlink.model.Vehicle

/**
  * Example:
  *
  * {{{
  * SYSTEM_TIME.time_unix_usec = 1606537752034000
  * SYSTEM_TIME.time_boot_ms = 219978456
  * ...
  * HIL_SENSOR.time_usec = 1606536869627000
  * HIL_SENSOR.xacc = 0.11460307
  * HIL_SENSOR.yacc = -0.0042598564
  * HIL_SENSOR.zacc = -9.768643
  * HIL_SENSOR.xgyro = -0.006957794
  * HIL_SENSOR.ygyro = 8.2095177E-4
  * HIL_SENSOR.zgyro = 0.014295125
  * HIL_SENSOR.xmag = 0.22091612
  * HIL_SENSOR.ymag = 0.011122807
  * HIL_SENSOR.zmag = 0.42089325
  * HIL_SENSOR.abs_pressure = 954.20074
  * HIL_SENSOR.pressure_alt = 503.6774
  * HIL_STATE_QUATERNION.time_usec = 1606536869627000
  * HIL_STATE_QUATERNION.attitude_quaternion = [1.0, 0.0, 0.0, 0.0]
  * HIL_STATE_QUATERNION.lat = 473979961
  * HIL_STATE_QUATERNION.lon = 85455361
  * HIL_STATE_QUATERNION.true_airspeed = 482
  * HIL_GPS.time_usec = 1606536869627000
  * HIL_GPS.lat = 473979989
  * HIL_GPS.lon = 85455240
  * HIL_GPS.alt = 508977
  * HIL_GPS.eph = 1157
  * HIL_GPS.epv = 1166
  * }}}
  *
  * @author Peter Potts
  */
object AutopilotInputState {

  /**
    * {{{
    * timeUnixUSec Timestamp of the master clock in microseconds since UNIX epoch.
    * timeBootMS   Timestamp of the component clock since boot time in milliseconds.
    * }}}
    */
  object VarSystemTime extends VarMessage[Vehicle.SystemTime](1)

  /**
    * {{{
    * timeUSec      Timestamp (microseconds, synced to UNIX time or since system boot)
    * xAcc          X acceleration (m/s^^2)
    * yAcc          Y acceleration (m/s^^2)
    * zAcc          Z acceleration (m/s^^2)
    * xGyro         Angular speed around X axis in body frame (rad / sec)
    * yGyro         Angular speed around Y axis in body frame (rad / sec)
    * zGyro         Angular speed around Z axis in body frame (rad / sec)
    * xMag          X Magnetic field (Gauss)
    * yMag          Y Magnetic field (Gauss)
    * zMag          Z Magnetic field (Gauss)
    * absPressure   Absolute pressure in millibar
    * diffPressure  Differential pressure (airspeed) in millibar
    * pressureAlt   Altitude calculated from pressure
    * temperature   Temperature in degrees celsius
    * fieldsUpdated Bitmask for fields that have updated since last message,
    *               bit 0 = xacc,
    *               bit 12: temperature,
    *               bit 31: full reset of attitude/position/velocities/etc
    *               was performed in sim.
    * }}}
    */
  object VarHILSensor extends VarMessage[Vehicle.HILSensor](1)

  /**
    * {{{
    * timeUSec          Timestamp (microseconds since UNIX epoch or
    *                   microseconds since system boot)
    * fixType           0-1: no fix, 2: 2D fix, 3: 3D fix.
    *                   Some applications will not use the value of this field
    *                   unless it is at least two,
    *                   so always correctly fill in the fix.
    * lat               Latitude (WGS84), in degrees * 1E7
    * lon               Longitude (WGS84), in degrees * 1E7
    * alt               Altitude (AMSL, not WGS84), in meters * 1000 (positive for up)
    * epH               GPS HDOP horizontal dilution of position in cm (m*100).
    *                   If unknown, set to: 65535
    * epV               GPS VDOP vertical dilution of position in cm (m*100).
    *                   If unknown, set to: 65535
    * vel               GPS ground speed in cm/s. If unknown, set to: 65535
    * vN                GPS velocity in cm/s in NORTH direction
    *                   in earth-fixed NED frame
    * vE                GPS velocity in cm/s in EAST direction
    *                   in earth-fixed NED frame
    * vD                GPS velocity in cm/s in DOWN direction
    *                   in earth-fixed NED frame
    * cOG               Course over ground (NOT heading, but direction of movement)
    *                   in degrees * 100, 0.0..359.99 degrees.
    *                   If unknown, set to: 65535
    * satellitesVisible Number of satellites visible. If unknown, set to 255
    * }}}
    */
  object VarHILGPS extends VarMessage[Vehicle.HILGPS](1)

  /**
    * {{{
    * timeUSec           Timestamp (microseconds since UNIX epoch or
    *                    microseconds since system boot)
    * attitudeQuaternion Vehicle attitude expressed as normalized quaternion
    *                    in w, x, y, z order (with 1 0 0 0 being the null-rotation)
    * rollSpeed          Body frame roll / phi angular speed (rad/s)
    * pitchSpeed         Body frame pitch / theta angular speed (rad/s)
    * yawSpeed           Body frame yaw / psi angular speed (rad/s)
    * lat                Latitude, expressed as * 1E7
    * lon                Longitude, expressed as * 1E7
    * alt                Altitude in meters, expressed as * 1000 (millimeters)
    * vX                 Ground X Speed (Latitude), expressed as cm/s
    * vY                 Ground Y Speed (Longitude), expressed as cm/s
    * vZ                 Ground Z Speed (Altitude), expressed as cm/s
    * indAirspeed        Indicated airspeed, expressed as cm/s
    * trueAirspeed       True airspeed, expressed as cm/s
    * xAcc               X acceleration (mg)
    * yAcc               Y acceleration (mg)
    * zAcc               Z acceleration (mg)
    * }}}
    */
  object VarHILStateQuaternion extends VarMessage[Vehicle.HILStateQuaternion](1)

}
