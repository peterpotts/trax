package com.peterpotts.trax.mavlink.autopilot

import com.peterpotts.trax.mavlink.model.MAVEnums.MAVLandedState
import com.peterpotts.trax.mavlink.model.{GroundControl, Vehicle}

/**
  * {{{
  * The raw value is the standard PPM modulation in microseconds.
  * 1000 microseconds is 0% and 2000 microseconds is 100%.
  * The control outputs range from -1 to 1.
  *
  * 69 MANUAL_CONTROL
  * Manually control the vehicle using standard joystick axes.
  *
  * 92 HIL_RC_INPUTS_RAW
  * Raw values sent from vehicle to autopilot.
  * The RAW values of the RC channels received.
  *
  * 36 SERVO_OUTPUT_RAW
  * Raw values sent from autopilot to ground control.
  * The RAW values of the servo outputs.
  *
  * 65 RC_CHANNELS
  * Raw values sent from autopilot to ground control.
  * The PPM values of the RC channels received.
  *
  * 93 HIL_ACTUATOR_CONTROLS
  * Control outputs sent from autopilot to vehicle.
  * Hardware in the loop control outputs.
  *
  * RSSI
  * Receive signal strength indicator, 0: 0%, 255: 100%
  * }}}
  *
  * @author Peter Potts
  */
object AutopilotControlState {

  object VarLandedState extends Var[MAVLandedState](1)

  /**
    * {{{
    * timeUSec  Timestamp (microseconds since UNIX epoch or
    *           microseconds since system boot)
    * chan1Raw  RC channel 1 value, in microseconds
    * chan2Raw  RC channel 2 value, in microseconds
    * chan3Raw  RC channel 3 value, in microseconds
    * chan4Raw  RC channel 4 value, in microseconds
    * chan5Raw  RC channel 5 value, in microseconds
    * chan6Raw  RC channel 6 value, in microseconds
    * chan7Raw  RC channel 7 value, in microseconds
    * chan8Raw  RC channel 8 value, in microseconds
    * chan9Raw  RC channel 9 value, in microseconds
    * chan10Raw RC channel 10 value, in microseconds
    * chan11Raw RC channel 11 value, in microseconds
    * chan12Raw RC channel 12 value, in microseconds
    * rSSI      Receive signal strength indicator, 0: 0%, 255: 100%
    * }}}
    */
  object VarHILRCInputsRaw extends VarMessage[Vehicle.HILRCInputsRaw](2)

  /**
    * {{{
    * target  The system to be controlled.
    * x       X-axis, normalized to the range [-1000,1000].
    *         A value of INT16_MAX indicates that this axis is invalid.
    *         Generally corresponds to forward(1000)-backward(-1000) movement
    *         on a joystick and the pitch of a vehicle.
    * y       Y-axis, normalized to the range [-1000,1000].
    *         A value of INT16_MAX indicates that this axis is invalid.
    *         Generally corresponds to left(-1000)-right(1000) movement
    *         on a joystick and the roll of a vehicle.
    * z       Z-axis, normalized to the range [-1000,1000].
    *         A value of INT16_MAX indicates that this axis is invalid.
    *         Generally corresponds to a separate slider movement
    *         with maximum being 1000 and minimum being -1000
    *         on a joystick and the thrust of a vehicle.
    *         Positive values are positive thrust,
    *         negative values are negative thrust.
    * r       R-axis, normalized to the range [-1000,1000].
    *         A value of INT16_MAX indicates that this axis is invalid.
    *         Generally corresponds to a twisting of the joystick,
    *         with counter-clockwise being 1000 and
    *         clockwise being -1000, and
    *         the yaw of a vehicle.
    * buttons A bitfield corresponding to the joystick buttons' current state,
    *        1 for pressed, 0 for released.
    *        The lowest bit corresponds to Button 1.
    * }}}
    */
  object VarManualControl extends VarMessage[GroundControl.ManualControl](1)

}
