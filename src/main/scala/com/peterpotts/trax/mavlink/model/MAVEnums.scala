package com.peterpotts.trax.mavlink.model

/**
  * @author Peter Potts
  */
object MAVEnums {

  /**
    * MAV_AUTOPILOT
    *
    * Micro air vehicle / autopilot classes. This identifies the individual model.
    *
    * {{{
    * entry 0 GENERIC Generic autopilot, full support for everything
    * entry 1 RESERVED Reserved for future use.
    * entry 2 SLUGS SLUGS autopilot, http://slugsuav.soe.ucsc.edu
    * entry 3 ARDUPILOTMEGA ArduPilotMega / ArduCopter, http://diydrones.com
    * entry 4 OPENPILOT OpenPilot, http://openpilot.org
    * entry 5 GENERIC_WAYPOINTS_ONLY Generic autopilot only supporting simple waypoints
    * entry 6 GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY Generic autopilot supporting waypoints and other simple navigation commands
    * entry 7 GENERIC_MISSION_FULL Generic autopilot supporting the full mission command set
    * entry 8 INVALID No valid autopilot, e.g. a GCS or other MAVLink component
    * entry 9 PPZ PPZ UAV - http://nongnu.org/paparazzi
    * entry 10 UDB UAV Dev Board
    * entry 11 FP FlexiPilot
    * entry 12 PX4 PX4 Autopilot - http://pixhawk.ethz.ch/px4/
    * entry 13 SMACCMPILOT SMACCMPilot - http://smaccmpilot.org
    * entry 14 AUTOQUAD AutoQuad -- http://autoquad.org
    * entry 15 ARMAZILA Armazila -- http://armazila.com
    * entry 16 AEROB Aerob -- http://aerob.ru
    * entry 17 ASLUAV ASLUAV autopilot -- http://www.asl.ethz.ch
    * entry 18 SMARTAP SmartAP Autopilot - http://sky-drones.com
    * }}}
    */
  class MAVAutopilot(val value: Int)

  object MAVAutopilot {

    case object Generic extends MAVAutopilot(0)

    case object Reserved extends MAVAutopilot(1)

    case object Slugs extends MAVAutopilot(2)

    case object ArduPilotMega extends MAVAutopilot(3)

    case object OpenPilot extends MAVAutopilot(4)

    case object GenericWaypointsOnly extends MAVAutopilot(5)

    case object GenericWaypointsAndSimpleNavigationOnly extends MAVAutopilot(6)

    case object GenericMissionFull extends MAVAutopilot(7)

    case object Invalid extends MAVAutopilot(8)

    case object PPZ extends MAVAutopilot(9)

    case object UDB extends MAVAutopilot(10)

    case object FP extends MAVAutopilot(11)

    case object PX4 extends MAVAutopilot(12)

    case object SMACCMPilot extends MAVAutopilot(13)

    case object AutoQuad extends MAVAutopilot(14)

    case object Armazila extends MAVAutopilot(15)

    case object Aerob extends MAVAutopilot(16)

    case object ASLUAV extends MAVAutopilot(17)

    case object SmartAP extends MAVAutopilot(18)

    val set: Set[MAVAutopilot] =
      Set(
        Generic,
        Reserved,
        Slugs,
        ArduPilotMega,
        OpenPilot,
        GenericWaypointsOnly,
        GenericWaypointsAndSimpleNavigationOnly,
        GenericMissionFull,
        Invalid,
        PPZ,
        UDB,
        FP,
        PX4,
        SMACCMPilot,
        AutoQuad,
        Armazila,
        Aerob,
        ASLUAV,
        SmartAP
      )

    val map: Map[Int, MAVAutopilot] = set.toSeq.map(instance => instance.value -> instance).toMap

    def apply(value: Int): MAVAutopilot = map.getOrElse(value, new MAVAutopilot(value))

    def bitmask(value: Int): Set[MAVAutopilot] = {
      val known = set.filter(instance => (instance.value & value) == instance.value)
      val sum = known.map(_.value).sum
      if (value == sum) known else known + new MAVAutopilot(value - sum)
    }
  }

  /**
    * MAV_TYPE
    *
    * {{{
    * entry 0 GENERIC Generic micro air vehicle.
    * entry 1 FIXED_WING Fixed wing aircraft.
    * entry 2 QUADROTOR Quadrotor
    * entry 3 COAXIAL Coaxial helicopter
    * entry 4 HELICOPTER Normal helicopter with tail rotor.
    * entry 5 ANTENNA_TRACKER Ground installation
    * entry 6 GCS Operator control unit / ground control station
    * entry 7 AIRSHIP Airship, controlled
    * entry 8 FREE_BALLOON Free balloon, uncontrolled
    * entry 9 ROCKET Rocket
    * entry 10 GROUND_ROVER Ground rover
    * entry 11 SURFACE_BOAT Surface vessel, boat, ship
    * entry 12 SUBMARINE Submarine
    * entry 13 HEXAROTOR Hexarotor
    * entry 14 OCTOROTOR Octorotor
    * entry 15 TRICOPTER Tricopter
    * entry 16 FLAPPING_WING Flapping wing
    * entry 17 KITE Kite
    * entry 18 ONBOARD_CONTROLLER Onboard companion controller
    * entry 19 VTOL_DUOROTOR Two-rotor VTOL using control surfaces in vertical operation in addition. Tailsitter.
    * entry 20 VTOL_QUADROTOR Quad-rotor VTOL using a V-shaped quad config in vertical operation. Tailsitter.
    * entry 21 VTOL_TILTROTOR Tiltrotor VTOL
    * entry 22 VTOL_RESERVED2 VTOL reserved 2
    * entry 23 VTOL_RESERVED3 VTOL reserved 3
    * entry 24 VTOL_RESERVED4 VTOL reserved 4
    * entry 25 VTOL_RESERVED5 VTOL reserved 5
    * entry 26 GIMBAL Onboard gimbal
    * entry 27 ADSB Onboard ADSB peripheral
    * entry 28 PARAFOIL Steerable, nonrigid airfoil
    * }}}
    */
  class MAVType(val value: Int)

  object MAVType {

    case object Generic extends MAVType(0)

    case object FixedWing extends MAVType(1)

    case object Quadrotor extends MAVType(2)

    case object Coaxial extends MAVType(3)

    case object Helicopter extends MAVType(4)

    case object AntennaTracker extends MAVType(5)

    case object GCS extends MAVType(6)

    case object Airship extends MAVType(7)

    case object FreeBalloon extends MAVType(8)

    case object Rocket extends MAVType(9)

    case object GroundRover extends MAVType(10)

    case object SurfaceBoat extends MAVType(11)

    case object Submarine extends MAVType(12)

    case object Hexarotor extends MAVType(13)

    case object Octorotor extends MAVType(14)

    case object Tricopter extends MAVType(15)

    case object FlappingWing extends MAVType(16)

    case object Kite extends MAVType(17)

    case object OnboardController extends MAVType(18)

    case object VTOLDuorotor extends MAVType(19)

    case object VTOLQuadrotor extends MAVType(20)

    case object VTOLTiltrotor extends MAVType(21)

    case object VTOLReserved2 extends MAVType(22)

    case object VTOLReserved3 extends MAVType(23)

    case object VTOLReserved4 extends MAVType(24)

    case object VTOLReserved5 extends MAVType(25)

    case object Gimbal extends MAVType(26)

    case object ADSB extends MAVType(27)

    case object Parafoil extends MAVType(28)

    val set: Set[MAVType] =
      Set(
        Generic,
        FixedWing,
        Quadrotor,
        Coaxial,
        Helicopter,
        AntennaTracker,
        GCS,
        Airship,
        FreeBalloon,
        Rocket,
        GroundRover,
        SurfaceBoat,
        Submarine,
        Hexarotor,
        Octorotor,
        Tricopter,
        FlappingWing,
        Kite,
        OnboardController,
        VTOLDuorotor,
        VTOLQuadrotor,
        VTOLTiltrotor,
        VTOLReserved2,
        VTOLReserved3,
        VTOLReserved4,
        VTOLReserved5,
        Gimbal,
        ADSB,
        Parafoil
      )

    val map: Map[Int, MAVType] = set.toSeq.map(instance => instance.value -> instance).toMap

    def apply(value: Int): MAVType = map.getOrElse(value, new MAVType(value))

    def bitmask(value: Int): Set[MAVType] = {
      val known = set.filter(instance => (instance.value & value) == instance.value)
      val sum = known.map(_.value).sum
      if (value == sum) known else known + new MAVType(value - sum)
    }
  }

  /**
    * FIRMWARE_VERSION_TYPE
    *
    * These values define the type of firmware release.  These values indicate the first version or release of this type.  For example the first alpha release would be 64, the second would be 65.
    *
    * {{{
    * entry 0 DEV development release
    * entry 64 ALPHA alpha release
    * entry 128 BETA beta release
    * entry 192 RC release candidate
    * entry 255 OFFICIAL official stable release
    * }}}
    */
  class FirmwareVersionType(val value: Int)

  object FirmwareVersionType {

    case object Dev extends FirmwareVersionType(0)

    case object Alpha extends FirmwareVersionType(64)

    case object Beta extends FirmwareVersionType(128)

    case object RC extends FirmwareVersionType(192)

    case object Official extends FirmwareVersionType(255)

    val set: Set[FirmwareVersionType] =
      Set(
        Dev,
        Alpha,
        Beta,
        RC,
        Official
      )

    val map: Map[Int, FirmwareVersionType] = set.toSeq.map(instance => instance.value -> instance).toMap

    def apply(value: Int): FirmwareVersionType = map.getOrElse(value, new FirmwareVersionType(value))

    def bitmask(value: Int): Set[FirmwareVersionType] = {
      val known = set.filter(instance => (instance.value & value) == instance.value)
      val sum = known.map(_.value).sum
      if (value == sum) known else known + new FirmwareVersionType(value - sum)
    }
  }

  /**
    * MAV_MODE_FLAG
    *
    * These flags encode the MAV mode.
    *
    * {{{
    * entry 128 SAFETY_ARMED 0b10000000 MAV safety set to armed. Motors are enabled / running / can start. Ready to fly. Additional note: this flag is to be ignore when sent in the command MAV_CMD_DO_SET_MODE and MAV_CMD_COMPONENT_ARM_DISARM shall be used instead. The flag can still be used to report the armed state.
    * entry 64 MANUAL_INPUT_ENABLED 0b01000000 remote control input is enabled.
    * entry 32 HIL_ENABLED 0b00100000 hardware in the loop simulation. All motors / actuators are blocked, but internal software is full operational.
    * entry 16 STABILIZE_ENABLED 0b00010000 system stabilizes electronically its attitude (and optionally position). It needs however further control inputs to move around.
    * entry 8 GUIDED_ENABLED 0b00001000 guided mode enabled, system flies waypoints / mission items.
    * entry 4 AUTO_ENABLED 0b00000100 autonomous mode enabled, system finds its own goal positions. Guided flag can be set or not, depends on the actual implementation.
    * entry 2 TEST_ENABLED 0b00000010 system has a test mode enabled. This flag is intended for temporary system tests and should not be used for stable implementations.
    * entry 1 CUSTOM_MODE_ENABLED 0b00000001 Reserved for future use.
    * }}}
    */
  class MAVModeFlag(val value: Int)

  object MAVModeFlag {

    case object SafetyArmed extends MAVModeFlag(128)

    case object ManualInputEnabled extends MAVModeFlag(64)

    case object HILEnabled extends MAVModeFlag(32)

    case object StabilizeEnabled extends MAVModeFlag(16)

    case object GuidedEnabled extends MAVModeFlag(8)

    case object AutoEnabled extends MAVModeFlag(4)

    case object TestEnabled extends MAVModeFlag(2)

    case object CustomModeEnabled extends MAVModeFlag(1)

    val set: Set[MAVModeFlag] =
      Set(
        SafetyArmed,
        ManualInputEnabled,
        HILEnabled,
        StabilizeEnabled,
        GuidedEnabled,
        AutoEnabled,
        TestEnabled,
        CustomModeEnabled
      )

    val map: Map[Int, MAVModeFlag] = set.toSeq.map(instance => instance.value -> instance).toMap

    def apply(value: Int): MAVModeFlag = map.getOrElse(value, new MAVModeFlag(value))

    def bitmask(value: Int): Set[MAVModeFlag] = {
      val known = set.filter(instance => (instance.value & value) == instance.value)
      val sum = known.map(_.value).sum
      if (value == sum) known else known + new MAVModeFlag(value - sum)
    }
  }

  /**
    * MAV_MODE_FLAG_DECODE_POSITION
    *
    * These values encode the bit positions of the decode position. These values can be used to read the value of a flag bit by combining the base_mode variable with AND with the flag position value. The result will be either 0 or 1, depending on if the flag is set or not.
    *
    * {{{
    * entry 128 SAFETY First bit:  10000000
    * entry 64 MANUAL Second bit: 01000000
    * entry 32 HIL Third bit:  00100000
    * entry 16 STABILIZE Fourth bit: 00010000
    * entry 8 GUIDED Fifth bit:  00001000
    * entry 4 AUTO Sixt bit:   00000100
    * entry 2 TEST Seventh bit: 00000010
    * entry 1 CUSTOM_MODE Eighth bit: 00000001
    * }}}
    */
  class MAVModeFlagDecodePosition(val value: Int)

  object MAVModeFlagDecodePosition {

    case object Safety extends MAVModeFlagDecodePosition(128)

    case object Manual extends MAVModeFlagDecodePosition(64)

    case object HIL extends MAVModeFlagDecodePosition(32)

    case object Stabilize extends MAVModeFlagDecodePosition(16)

    case object Guided extends MAVModeFlagDecodePosition(8)

    case object Auto extends MAVModeFlagDecodePosition(4)

    case object Test extends MAVModeFlagDecodePosition(2)

    case object CustomMode extends MAVModeFlagDecodePosition(1)

    val set: Set[MAVModeFlagDecodePosition] =
      Set(
        Safety,
        Manual,
        HIL,
        Stabilize,
        Guided,
        Auto,
        Test,
        CustomMode
      )

    val map: Map[Int, MAVModeFlagDecodePosition] = set.toSeq.map(instance => instance.value -> instance).toMap

    def apply(value: Int): MAVModeFlagDecodePosition = map.getOrElse(value, new MAVModeFlagDecodePosition(value))

    def bitmask(value: Int): Set[MAVModeFlagDecodePosition] = {
      val known = set.filter(instance => (instance.value & value) == instance.value)
      val sum = known.map(_.value).sum
      if (value == sum) known else known + new MAVModeFlagDecodePosition(value - sum)
    }
  }

  /**
    * MAV_GOTO
    *
    * Override command, pauses current mission execution and moves immediately to a position
    *
    * {{{
    * entry 0 DO_HOLD Hold at the current position.
    * entry 1 DO_CONTINUE Continue with the next item in mission execution.
    * entry 2 HOLD_AT_CURRENT_POSITION Hold at the current position of the system
    * entry 3 HOLD_AT_SPECIFIED_POSITION Hold at the position specified in the parameters of the DO_HOLD action
    * }}}
    */
  class MAVGoto(val value: Int)

  object MAVGoto {

    case object DoHold extends MAVGoto(0)

    case object DoContinue extends MAVGoto(1)

    case object HoldAtCurrentPosition extends MAVGoto(2)

    case object HoldAtSpecifiedPosition extends MAVGoto(3)

    val set: Set[MAVGoto] =
      Set(
        DoHold,
        DoContinue,
        HoldAtCurrentPosition,
        HoldAtSpecifiedPosition
      )

    val map: Map[Int, MAVGoto] = set.toSeq.map(instance => instance.value -> instance).toMap

    def apply(value: Int): MAVGoto = map.getOrElse(value, new MAVGoto(value))

    def bitmask(value: Int): Set[MAVGoto] = {
      val known = set.filter(instance => (instance.value & value) == instance.value)
      val sum = known.map(_.value).sum
      if (value == sum) known else known + new MAVGoto(value - sum)
    }
  }

  /**
    * MAV_MODE
    *
    * These defines are predefined OR-combined mode flags. There is no need to use values from this enum, but it
    * simplifies the use of the mode flags. Note that manual input is enabled in all modes as a safety override.
    *
    * {{{
    * entry 0 PREFLIGHT System is not ready to fly, booting, calibrating, etc. No flag is set.
    * entry 80 STABILIZE_DISARMED System is allowed to be active, under assisted RC control.
    * entry 208 STABILIZE_ARMED System is allowed to be active, under assisted RC control.
    * entry 64 MANUAL_DISARMED System is allowed to be active, under manual (RC) control, no stabilization
    * entry 192 MANUAL_ARMED System is allowed to be active, under manual (RC) control, no stabilization
    * entry 88 GUIDED_DISARMED System is allowed to be active, under autonomous control, manual setpoint
    * entry 216 GUIDED_ARMED System is allowed to be active, under autonomous control, manual setpoint
    * entry 92 AUTO_DISARMED System is allowed to be active, under autonomous control and navigation (the trajectory is decided onboard and not pre-programmed by waypoints)
    * entry 220 AUTO_ARMED System is allowed to be active, under autonomous control and navigation (the trajectory is decided onboard and not pre-programmed by waypoints)
    * entry 66 TEST_DISARMED UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only.
    * entry 194 TEST_ARMED UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only.
    * }}}
    */
  class MAVMode(val value: Int)

  object MAVMode {

    case object Preflight extends MAVMode(0)

    case object StabilizeDisarmed extends MAVMode(80)

    case object StabilizeArmed extends MAVMode(208)

    case object ManualDisarmed extends MAVMode(64)

    case object ManualArmed extends MAVMode(192)

    case object GuidedDisarmed extends MAVMode(88)

    case object GuidedArmed extends MAVMode(216)

    case object AutoDisarmed extends MAVMode(92)

    case object AutoArmed extends MAVMode(220)

    case object TestDisarmed extends MAVMode(66)

    case object TestArmed extends MAVMode(194)

    val set: Set[MAVMode] =
      Set(
        Preflight,
        StabilizeDisarmed,
        StabilizeArmed,
        ManualDisarmed,
        ManualArmed,
        GuidedDisarmed,
        GuidedArmed,
        AutoDisarmed,
        AutoArmed,
        TestDisarmed,
        TestArmed
      )

    val map: Map[Int, MAVMode] = set.toSeq.map(instance => instance.value -> instance).toMap

    def apply(value: Int): MAVMode = map.getOrElse(value, new MAVMode(value))

    def bitmask(value: Int): Set[MAVMode] = {
      val known = set.filter(instance => (instance.value & value) == instance.value)
      val sum = known.map(_.value).sum
      if (value == sum) known else known + new MAVMode(value - sum)
    }
  }

  /**
    * MAV_STATE
    *
    * {{{
    * entry 0 UNINIT Uninitialized system, state is unknown.
    * entry 1 BOOT System is booting up.
    * entry 2 CALIBRATING System is calibrating and not flight-ready.
    * entry 3 STANDBY System is grounded and on standby. It can be launched any time.
    * entry 4 ACTIVE System is active and might be already airborne. Motors are engaged.
    * entry 5 CRITICAL System is in a non-normal flight mode. It can however still navigate.
    * entry 6 EMERGENCY System is in a non-normal flight mode. It lost control over parts or over the whole airframe. It is in mayday and going down.
    * entry 7 POWEROFF System just initialized its power-down sequence, will shut down now.
    * entry 8 FLIGHT_TERMINATION System is terminating itself.
    * }}}
    */
  class MAVState(val value: Int)

  object MAVState {

    case object Uninit extends MAVState(0)

    case object Boot extends MAVState(1)

    case object Calibrating extends MAVState(2)

    case object Standby extends MAVState(3)

    case object Active extends MAVState(4)

    case object Critical extends MAVState(5)

    case object Emergency extends MAVState(6)

    case object Poweroff extends MAVState(7)

    case object FlightTermination extends MAVState(8)

    val set: Set[MAVState] =
      Set(
        Uninit,
        Boot,
        Calibrating,
        Standby,
        Active,
        Critical,
        Emergency,
        Poweroff,
        FlightTermination
      )

    val map: Map[Int, MAVState] = set.toSeq.map(instance => instance.value -> instance).toMap

    def apply(value: Int): MAVState = map.getOrElse(value, new MAVState(value))

    def bitmask(value: Int): Set[MAVState] = {
      val known = set.filter(instance => (instance.value & value) == instance.value)
      val sum = known.map(_.value).sum
      if (value == sum) known else known + new MAVState(value - sum)
    }
  }

  /**
    * MAV_COMPONENT
    *
    * {{{
    * entry 0 ID_ALL
    * entry 1 ID_AUTOPILOT1
    * entry 100 ID_CAMERA
    * entry 101 ID_CAMERA2
    * entry 102 ID_CAMERA3
    * entry 103 ID_CAMERA4
    * entry 104 ID_CAMERA5
    * entry 105 ID_CAMERA6
    * entry 140 ID_SERVO1
    * entry 141 ID_SERVO2
    * entry 142 ID_SERVO3
    * entry 143 ID_SERVO4
    * entry 144 ID_SERVO5
    * entry 145 ID_SERVO6
    * entry 146 ID_SERVO7
    * entry 147 ID_SERVO8
    * entry 148 ID_SERVO9
    * entry 149 ID_SERVO10
    * entry 150 ID_SERVO11
    * entry 151 ID_SERVO12
    * entry 152 ID_SERVO13
    * entry 153 ID_SERVO14
    * entry 154 ID_GIMBAL
    * entry 155 ID_LOG
    * entry 156 ID_ADSB
    * entry 157 ID_OSD On Screen Display (OSD) devices for video links
    * entry 158 ID_PERIPHERAL Generic autopilot peripheral component ID. Meant for devices that do not implement the parameter sub-protocol
    * entry 159 ID_QX1_GIMBAL
    * entry 180 ID_MAPPER
    * entry 190 ID_MISSIONPLANNER
    * entry 195 ID_PATHPLANNER
    * entry 200 ID_IMU
    * entry 201 ID_IMU_2
    * entry 202 ID_IMU_3
    * entry 220 ID_GPS
    * entry 221 ID_GPS2
    * entry 240 ID_UDP_BRIDGE
    * entry 241 ID_UART_BRIDGE
    * entry 250 ID_SYSTEM_CONTROL
    * }}}
    */
  class MAVComponent(val value: Int)

  object MAVComponent {

    case object IdAll extends MAVComponent(0)

    case object IdAutopilot1 extends MAVComponent(1)

    case object IdCamera extends MAVComponent(100)

    case object IdCamera2 extends MAVComponent(101)

    case object IdCamera3 extends MAVComponent(102)

    case object IdCamera4 extends MAVComponent(103)

    case object IdCamera5 extends MAVComponent(104)

    case object IdCamera6 extends MAVComponent(105)

    case object IdServo1 extends MAVComponent(140)

    case object IdServo2 extends MAVComponent(141)

    case object IdServo3 extends MAVComponent(142)

    case object IdServo4 extends MAVComponent(143)

    case object IdServo5 extends MAVComponent(144)

    case object IdServo6 extends MAVComponent(145)

    case object IdServo7 extends MAVComponent(146)

    case object IdServo8 extends MAVComponent(147)

    case object IdServo9 extends MAVComponent(148)

    case object IdServo10 extends MAVComponent(149)

    case object IdServo11 extends MAVComponent(150)

    case object IdServo12 extends MAVComponent(151)

    case object IdServo13 extends MAVComponent(152)

    case object IdServo14 extends MAVComponent(153)

    case object IdGimbal extends MAVComponent(154)

    case object IdLog extends MAVComponent(155)

    case object IdADSB extends MAVComponent(156)

    case object IdOSD extends MAVComponent(157)

    case object IdPeripheral extends MAVComponent(158)

    case object IdQX1Gimbal extends MAVComponent(159)

    case object IdMapper extends MAVComponent(180)

    case object IdMissionPlanner extends MAVComponent(190)

    case object IdPathPlanner extends MAVComponent(195)

    case object IdIMU extends MAVComponent(200)

    case object IdIMU2 extends MAVComponent(201)

    case object IdIMU3 extends MAVComponent(202)

    case object IdGPS extends MAVComponent(220)

    case object IdGPS2 extends MAVComponent(221)

    case object IdUDPBridge extends MAVComponent(240)

    case object IdUARTBridge extends MAVComponent(241)

    case object IdSystemControl extends MAVComponent(250)

    val set: Set[MAVComponent] =
      Set(
        IdAll,
        IdAutopilot1,
        IdCamera,
        IdCamera2,
        IdCamera3,
        IdCamera4,
        IdCamera5,
        IdCamera6,
        IdServo1,
        IdServo2,
        IdServo3,
        IdServo4,
        IdServo5,
        IdServo6,
        IdServo7,
        IdServo8,
        IdServo9,
        IdServo10,
        IdServo11,
        IdServo12,
        IdServo13,
        IdServo14,
        IdGimbal,
        IdLog,
        IdADSB,
        IdOSD,
        IdPeripheral,
        IdQX1Gimbal,
        IdMapper,
        IdMissionPlanner,
        IdPathPlanner,
        IdIMU,
        IdIMU2,
        IdIMU3,
        IdGPS,
        IdGPS2,
        IdUDPBridge,
        IdUARTBridge,
        IdSystemControl
      )

    val map: Map[Int, MAVComponent] = set.toSeq.map(instance => instance.value -> instance).toMap

    def apply(value: Int): MAVComponent = map.getOrElse(value, new MAVComponent(value))

    def bitmask(value: Int): Set[MAVComponent] = {
      val known = set.filter(instance => (instance.value & value) == instance.value)
      val sum = known.map(_.value).sum
      if (value == sum) known else known + new MAVComponent(value - sum)
    }
  }

  /**
    * MAV_SYS_STATUS_SENSOR
    *
    * These encode the sensors whose status is sent as part of the SYS_STATUS message.
    *
    * {{{
    * entry 1 SENSOR_3D_GYRO 0x01 3D gyro
    * entry 2 SENSOR_3D_ACCEL 0x02 3D accelerometer
    * entry 4 SENSOR_3D_MAG 0x04 3D magnetometer
    * entry 8 SENSOR_ABSOLUTE_PRESSURE 0x08 absolute pressure
    * entry 16 SENSOR_DIFFERENTIAL_PRESSURE 0x10 differential pressure
    * entry 32 SENSOR_GPS 0x20 GPS
    * entry 64 SENSOR_OPTICAL_FLOW 0x40 optical flow
    * entry 128 SENSOR_VISION_POSITION 0x80 computer vision position
    * entry 256 SENSOR_LASER_POSITION 0x100 laser based position
    * entry 512 SENSOR_EXTERNAL_GROUND_TRUTH 0x200 external ground truth (Vicon or Leica)
    * entry 1024 SENSOR_ANGULAR_RATE_CONTROL 0x400 3D angular rate control
    * entry 2048 SENSOR_ATTITUDE_STABILIZATION 0x800 attitude stabilization
    * entry 4096 SENSOR_YAW_POSITION 0x1000 yaw position
    * entry 8192 SENSOR_Z_ALTITUDE_CONTROL 0x2000 z/altitude control
    * entry 16384 SENSOR_XY_POSITION_CONTROL 0x4000 x/y position control
    * entry 32768 SENSOR_MOTOR_OUTPUTS 0x8000 motor outputs / control
    * entry 65536 SENSOR_RC_RECEIVER 0x10000 rc receiver
    * entry 131072 SENSOR_3D_GYRO2 0x20000 2nd 3D gyro
    * entry 262144 SENSOR_3D_ACCEL2 0x40000 2nd 3D accelerometer
    * entry 524288 SENSOR_3D_MAG2 0x80000 2nd 3D magnetometer
    * entry 1048576 GEOFENCE 0x100000 geofence
    * entry 2097152 AHRS 0x200000 AHRS subsystem health
    * entry 4194304 TERRAIN 0x400000 Terrain subsystem health
    * entry 8388608 REVERSE_MOTOR 0x800000 Motors are reversed
    * entry 16777216 LOGGING 0x1000000 Logging
    * entry 33554432 SENSOR_BATTERY 0x2000000 Battery
    * }}}
    */
  class MAVSysStatusSensor(val value: Int)

  object MAVSysStatusSensor {

    case object Sensor3dGyro extends MAVSysStatusSensor(1)

    case object Sensor3dAccel extends MAVSysStatusSensor(2)

    case object Sensor3dMag extends MAVSysStatusSensor(4)

    case object SensorAbsolutePressure extends MAVSysStatusSensor(8)

    case object SensorDifferentialPressure extends MAVSysStatusSensor(16)

    case object SensorGPS extends MAVSysStatusSensor(32)

    case object SensorOpticalFlow extends MAVSysStatusSensor(64)

    case object SensorVisionPosition extends MAVSysStatusSensor(128)

    case object SensorLaserPosition extends MAVSysStatusSensor(256)

    case object SensorExternalGroundTruth extends MAVSysStatusSensor(512)

    case object SensorAngularRateControl extends MAVSysStatusSensor(1024)

    case object SensorAttitudeStabilization extends MAVSysStatusSensor(2048)

    case object SensorYawPosition extends MAVSysStatusSensor(4096)

    case object SensorZAltitudeControl extends MAVSysStatusSensor(8192)

    case object SensorXYPositionControl extends MAVSysStatusSensor(16384)

    case object SensorMotorOutputs extends MAVSysStatusSensor(32768)

    case object SensorRCReceiver extends MAVSysStatusSensor(65536)

    case object Sensor3dGyro2 extends MAVSysStatusSensor(131072)

    case object Sensor3dAccel2 extends MAVSysStatusSensor(262144)

    case object Sensor3dMag2 extends MAVSysStatusSensor(524288)

    case object Geofence extends MAVSysStatusSensor(1048576)

    case object AHRS extends MAVSysStatusSensor(2097152)

    case object Terrain extends MAVSysStatusSensor(4194304)

    case object ReverseMotor extends MAVSysStatusSensor(8388608)

    case object Logging extends MAVSysStatusSensor(16777216)

    case object SensorBattery extends MAVSysStatusSensor(33554432)

    val set: Set[MAVSysStatusSensor] =
      Set(
        Sensor3dGyro,
        Sensor3dAccel,
        Sensor3dMag,
        SensorAbsolutePressure,
        SensorDifferentialPressure,
        SensorGPS,
        SensorOpticalFlow,
        SensorVisionPosition,
        SensorLaserPosition,
        SensorExternalGroundTruth,
        SensorAngularRateControl,
        SensorAttitudeStabilization,
        SensorYawPosition,
        SensorZAltitudeControl,
        SensorXYPositionControl,
        SensorMotorOutputs,
        SensorRCReceiver,
        Sensor3dGyro2,
        Sensor3dAccel2,
        Sensor3dMag2,
        Geofence,
        AHRS,
        Terrain,
        ReverseMotor,
        Logging,
        SensorBattery
      )

    val map: Map[Int, MAVSysStatusSensor] = set.toSeq.map(instance => instance.value -> instance).toMap

    def apply(value: Int): MAVSysStatusSensor = map.getOrElse(value, new MAVSysStatusSensor(value))

    def bitmask(value: Int): Set[MAVSysStatusSensor] = {
      val known = set.filter(instance => (instance.value & value) == instance.value)
      val sum = known.map(_.value).sum
      if (value == sum) known else known + new MAVSysStatusSensor(value - sum)
    }
  }

  /**
    * MAV_FRAME
    *
    * {{{
    * entry 0 GLOBAL Global coordinate frame, WGS84 coordinate system. First value / x: latitude, second value / y: longitude, third value / z: positive altitude over mean sea level (MSL)
    * entry 1 LOCAL_NED Local coordinate frame, Z-up (x: north, y: east, z: down).
    * entry 2 MISSION NOT a coordinate frame, indicates a mission command.
    * entry 3 GLOBAL_RELATIVE_ALT Global coordinate frame, WGS84 coordinate system, relative altitude over ground with respect to the home position. First value / x: latitude, second value / y: longitude, third value / z: positive altitude with 0 being at the altitude of the home location.
    * entry 4 LOCAL_ENU Local coordinate frame, Z-down (x: east, y: north, z: up)
    * entry 5 GLOBAL_INT Global coordinate frame, WGS84 coordinate system. First value / x: latitude in degrees*1.0e-7, second value / y: longitude in degrees*1.0e-7, third value / z: positive altitude over mean sea level (MSL)
    * entry 6 GLOBAL_RELATIVE_ALT_INT Global coordinate frame, WGS84 coordinate system, relative altitude over ground with respect to the home position. First value / x: latitude in degrees*10e-7, second value / y: longitude in degrees*10e-7, third value / z: positive altitude with 0 being at the altitude of the home location.
    * entry 7 LOCAL_OFFSET_NED Offset to the current local frame. Anything expressed in this frame should be added to the current local frame position.
    * entry 8 BODY_NED Setpoint in body NED frame. This makes sense if all position control is externalized - e.g. useful to command 2 m/s^2 acceleration to the right.
    * entry 9 BODY_OFFSET_NED Offset in body NED frame. This makes sense if adding setpoints to the current flight path, to avoid an obstacle - e.g. useful to command 2 m/s^2 acceleration to the east.
    * entry 10 GLOBAL_TERRAIN_ALT Global coordinate frame with above terrain level altitude. WGS84 coordinate system, relative altitude over terrain with respect to the waypoint coordinate. First value / x: latitude in degrees, second value / y: longitude in degrees, third value / z: positive altitude in meters with 0 being at ground level in terrain model.
    * entry 11 GLOBAL_TERRAIN_ALT_INT Global coordinate frame with above terrain level altitude. WGS84 coordinate system, relative altitude over terrain with respect to the waypoint coordinate. First value / x: latitude in degrees*10e-7, second value / y: longitude in degrees*10e-7, third value / z: positive altitude in meters with 0 being at ground level in terrain model.
    * }}}
    */
  class MAVFrame(val value: Int)

  object MAVFrame {

    case object Global extends MAVFrame(0)

    case object LocalNED extends MAVFrame(1)

    case object Mission extends MAVFrame(2)

    case object GlobalRelativeAlt extends MAVFrame(3)

    case object LocalENU extends MAVFrame(4)

    case object GlobalInt extends MAVFrame(5)

    case object GlobalRelativeAltInt extends MAVFrame(6)

    case object LocalOffsetNED extends MAVFrame(7)

    case object BodyNED extends MAVFrame(8)

    case object BodyOffsetNED extends MAVFrame(9)

    case object GlobalTerrainAlt extends MAVFrame(10)

    case object GlobalTerrainAltInt extends MAVFrame(11)

    val set: Set[MAVFrame] =
      Set(
        Global,
        LocalNED,
        Mission,
        GlobalRelativeAlt,
        LocalENU,
        GlobalInt,
        GlobalRelativeAltInt,
        LocalOffsetNED,
        BodyNED,
        BodyOffsetNED,
        GlobalTerrainAlt,
        GlobalTerrainAltInt
      )

    val map: Map[Int, MAVFrame] = set.toSeq.map(instance => instance.value -> instance).toMap

    def apply(value: Int): MAVFrame = map.getOrElse(value, new MAVFrame(value))

    def bitmask(value: Int): Set[MAVFrame] = {
      val known = set.filter(instance => (instance.value & value) == instance.value)
      val sum = known.map(_.value).sum
      if (value == sum) known else known + new MAVFrame(value - sum)
    }
  }

  /**
    * MAVLINK_DATA_STREAM_TYPE
    *
    * {{{
    * entry 1 IMG_JPEG
    * entry 2 IMG_BMP
    * entry 3 IMG_RAW8U
    * entry 4 IMG_RAW32U
    * entry 5 IMG_PGM
    * entry 6 IMG_PNG
    * }}}
    */
  class MAVLinkDataStreamType(val value: Int)

  object MAVLinkDataStreamType {

    case object ImgJPEG extends MAVLinkDataStreamType(1)

    case object ImgBMP extends MAVLinkDataStreamType(2)

    case object ImgRaw8u extends MAVLinkDataStreamType(3)

    case object ImgRaw32u extends MAVLinkDataStreamType(4)

    case object ImgPGM extends MAVLinkDataStreamType(5)

    case object ImgPNG extends MAVLinkDataStreamType(6)

    val set: Set[MAVLinkDataStreamType] =
      Set(
        ImgJPEG,
        ImgBMP,
        ImgRaw8u,
        ImgRaw32u,
        ImgPGM,
        ImgPNG
      )

    val map: Map[Int, MAVLinkDataStreamType] = set.toSeq.map(instance => instance.value -> instance).toMap

    def apply(value: Int): MAVLinkDataStreamType = map.getOrElse(value, new MAVLinkDataStreamType(value))

    def bitmask(value: Int): Set[MAVLinkDataStreamType] = {
      val known = set.filter(instance => (instance.value & value) == instance.value)
      val sum = known.map(_.value).sum
      if (value == sum) known else known + new MAVLinkDataStreamType(value - sum)
    }
  }

  /**
    * FENCE_ACTION
    *
    * {{{
    * entry 0 NONE Disable fenced mode
    * entry 1 GUIDED Switched to guided mode to return point (fence point 0)
    * entry 2 REPORT Report fence breach, but don't take action
    * entry 3 GUIDED_THR_PASS Switched to guided mode to return point (fence point 0) with manual throttle control
    * entry 4 RTL Switch to RTL (return to launch) mode and head for the return point.
    * }}}
    */
  class FenceAction(val value: Int)

  object FenceAction {

    case object None extends FenceAction(0)

    case object Guided extends FenceAction(1)

    case object Report extends FenceAction(2)

    case object GuidedThrPass extends FenceAction(3)

    case object RTL extends FenceAction(4)

    val set: Set[FenceAction] =
      Set(
        None,
        Guided,
        Report,
        GuidedThrPass,
        RTL
      )

    val map: Map[Int, FenceAction] = set.toSeq.map(instance => instance.value -> instance).toMap

    def apply(value: Int): FenceAction = map.getOrElse(value, new FenceAction(value))

    def bitmask(value: Int): Set[FenceAction] = {
      val known = set.filter(instance => (instance.value & value) == instance.value)
      val sum = known.map(_.value).sum
      if (value == sum) known else known + new FenceAction(value - sum)
    }
  }

  /**
    * FENCE_BREACH
    *
    * {{{
    * entry 0 NONE No last fence breach
    * entry 1 MINALT Breached minimum altitude
    * entry 2 MAXALT Breached maximum altitude
    * entry 3 BOUNDARY Breached fence boundary
    * }}}
    */
  class FenceBreach(val value: Int)

  object FenceBreach {

    case object None extends FenceBreach(0)

    case object MinAlt extends FenceBreach(1)

    case object MaxAlt extends FenceBreach(2)

    case object Boundary extends FenceBreach(3)

    val set: Set[FenceBreach] =
      Set(
        None,
        MinAlt,
        MaxAlt,
        Boundary
      )

    val map: Map[Int, FenceBreach] = set.toSeq.map(instance => instance.value -> instance).toMap

    def apply(value: Int): FenceBreach = map.getOrElse(value, new FenceBreach(value))

    def bitmask(value: Int): Set[FenceBreach] = {
      val known = set.filter(instance => (instance.value & value) == instance.value)
      val sum = known.map(_.value).sum
      if (value == sum) known else known + new FenceBreach(value - sum)
    }
  }

  /**
    * MAV_MOUNT_MODE
    *
    * Enumeration of possible mount operation modes
    *
    * {{{
    * entry 0 RETRACT Load and keep safe position (Roll,Pitch,Yaw) from permant memory and stop stabilization
    * entry 1 NEUTRAL Load and keep neutral position (Roll,Pitch,Yaw) from permanent memory.
    * entry 2 MAVLINK_TARGETING Load neutral position and start MAVLink Roll,Pitch,Yaw control with stabilization
    * entry 3 RC_TARGETING Load neutral position and start RC Roll,Pitch,Yaw control with stabilization
    * entry 4 GPS_POINT Load neutral position and start to point to Lat,Lon,Alt
    * }}}
    */
  class MAVMountMode(val value: Int)

  object MAVMountMode {

    case object Retract extends MAVMountMode(0)

    case object Neutral extends MAVMountMode(1)

    case object MAVLinkTargeting extends MAVMountMode(2)

    case object RCTargeting extends MAVMountMode(3)

    case object GPSPoint extends MAVMountMode(4)

    val set: Set[MAVMountMode] =
      Set(
        Retract,
        Neutral,
        MAVLinkTargeting,
        RCTargeting,
        GPSPoint
      )

    val map: Map[Int, MAVMountMode] = set.toSeq.map(instance => instance.value -> instance).toMap

    def apply(value: Int): MAVMountMode = map.getOrElse(value, new MAVMountMode(value))

    def bitmask(value: Int): Set[MAVMountMode] = {
      val known = set.filter(instance => (instance.value & value) == instance.value)
      val sum = known.map(_.value).sum
      if (value == sum) known else known + new MAVMountMode(value - sum)
    }
  }

  /**
    * UAVCAN_NODE_HEALTH
    *
    * Generalized UAVCAN node health
    *
    * {{{
    * entry 0 OK The node is functioning properly.
    * entry 1 WARNING A critical parameter went out of range or the node has encountered a minor failure.
    * entry 2 ERROR The node has encountered a major failure.
    * entry 3 CRITICAL The node has suffered a fatal malfunction.
    * }}}
    */
  class UAVCANNodeHealth(val value: Int)

  object UAVCANNodeHealth {

    case object OK extends UAVCANNodeHealth(0)

    case object Warning extends UAVCANNodeHealth(1)

    case object Error extends UAVCANNodeHealth(2)

    case object Critical extends UAVCANNodeHealth(3)

    val set: Set[UAVCANNodeHealth] =
      Set(
        OK,
        Warning,
        Error,
        Critical
      )

    val map: Map[Int, UAVCANNodeHealth] = set.toSeq.map(instance => instance.value -> instance).toMap

    def apply(value: Int): UAVCANNodeHealth = map.getOrElse(value, new UAVCANNodeHealth(value))

    def bitmask(value: Int): Set[UAVCANNodeHealth] = {
      val known = set.filter(instance => (instance.value & value) == instance.value)
      val sum = known.map(_.value).sum
      if (value == sum) known else known + new UAVCANNodeHealth(value - sum)
    }
  }

  /**
    * UAVCAN_NODE_MODE
    *
    * Generalized UAVCAN node mode
    *
    * {{{
    * entry 0 OPERATIONAL The node is performing its primary functions.
    * entry 1 INITIALIZATION The node is initializing; this mode is entered immediately after startup.
    * entry 2 MAINTENANCE The node is under maintenance.
    * entry 3 SOFTWARE_UPDATE The node is in the process of updating its software.
    * entry 7 OFFLINE The node is no longer available online.
    * }}}
    */
  class UAVCANNodeMode(val value: Int)

  object UAVCANNodeMode {

    case object Operational extends UAVCANNodeMode(0)

    case object Initialization extends UAVCANNodeMode(1)

    case object Maintenance extends UAVCANNodeMode(2)

    case object SoftwareUpdate extends UAVCANNodeMode(3)

    case object Offline extends UAVCANNodeMode(7)

    val set: Set[UAVCANNodeMode] =
      Set(
        Operational,
        Initialization,
        Maintenance,
        SoftwareUpdate,
        Offline
      )

    val map: Map[Int, UAVCANNodeMode] = set.toSeq.map(instance => instance.value -> instance).toMap

    def apply(value: Int): UAVCANNodeMode = map.getOrElse(value, new UAVCANNodeMode(value))

    def bitmask(value: Int): Set[UAVCANNodeMode] = {
      val known = set.filter(instance => (instance.value & value) == instance.value)
      val sum = known.map(_.value).sum
      if (value == sum) known else known + new UAVCANNodeMode(value - sum)
    }
  }

  /**
    * MAV_CMD
    *
    * Commands to be executed by the MAV. They can be executed on user request, or as part of a mission script. If the action is used in a mission, the parameter mapping to the waypoint/mission message is as follows: Param 1, Param 2, Param 3, Param 4, X: Param 5, Y:Param 6, Z:Param 7. This command list is similar what ARINC 424 is for commercial aircraft: A data format how to interpret waypoint/mission data.
    *
    * {{{
    * entry 16 NAV_WAYPOINT Navigate to waypoint.
    * entry 17 NAV_LOITER_UNLIM Loiter around this waypoint an unlimited amount of time
    * entry 18 NAV_LOITER_TURNS Loiter around this waypoint for X turns
    * entry 19 NAV_LOITER_TIME Loiter around this waypoint for X seconds
    * entry 20 NAV_RETURN_TO_LAUNCH Return to launch location
    * entry 21 NAV_LAND Land at location
    * entry 22 NAV_TAKEOFF Takeoff from ground / hand
    * entry 23 NAV_LAND_LOCAL Land at local position (local frame only)
    * entry 24 NAV_TAKEOFF_LOCAL Takeoff from local position (local frame only)
    * entry 25 NAV_FOLLOW Vehicle following, i.e. this waypoint represents the position of a moving vehicle
    * entry 30 NAV_CONTINUE_AND_CHANGE_ALT Continue on the current course and climb/descend to specified altitude.  When the altitude is reached continue to the next command (i.e., don't proceed to the next command until the desired altitude is reached.
    * entry 31 NAV_LOITER_TO_ALT Begin loiter at the specified Latitude and Longitude.  If Lat=Lon=0, then loiter at the current position.  Don't consider the navigation command complete (don't leave loiter) until the altitude has been reached.  Additionally, if the Heading Required parameter is non-zero the  aircraft will not leave the loiter until heading toward the next waypoint.
    * entry 32 DO_FOLLOW Being following a target
    * entry 33 DO_FOLLOW_REPOSITION Reposition the MAV after a follow target command has been sent
    * entry 80 NAV_ROI Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras.
    * entry 81 NAV_PATHPLANNING Control autonomous path planning on the MAV.
    * entry 82 NAV_SPLINE_WAYPOINT Navigate to waypoint using a spline path.
    * entry 84 NAV_VTOL_TAKEOFF Takeoff from ground using VTOL mode
    * entry 85 NAV_VTOL_LAND Land using VTOL mode
    * entry 92 NAV_GUIDED_ENABLE hand control over to an external controller
    * entry 93 NAV_DELAY Delay the next navigation command a number of seconds or until a specified time
    * entry 94 NAV_PAYLOAD_PLACE Descend and place payload.  Vehicle descends until it detects a hanging payload has reached the ground, the gripper is opened to release the payload
    * entry 95 NAV_LAST NOP - This command is only used to mark the upper limit of the NAV/ACTION commands in the enumeration
    * entry 112 CONDITION_DELAY Delay mission state machine.
    * entry 113 CONDITION_CHANGE_ALT Ascend/descend at rate.  Delay mission state machine until desired altitude reached.
    * entry 114 CONDITION_DISTANCE Delay mission state machine until within desired distance of next NAV point.
    * entry 115 CONDITION_YAW Reach a certain target angle.
    * entry 159 CONDITION_LAST NOP - This command is only used to mark the upper limit of the CONDITION commands in the enumeration
    * entry 176 DO_SET_MODE Set system mode.
    * entry 177 DO_JUMP Jump to the desired command in the mission list.  Repeat this action only the specified number of times
    * entry 178 DO_CHANGE_SPEED Change speed and/or throttle set points.
    * entry 179 DO_SET_HOME Changes the home location either to the current location or a specified location.
    * entry 180 DO_SET_PARAMETER Set a system parameter.  Caution!  Use of this command requires knowledge of the numeric enumeration value of the parameter.
    * entry 181 DO_SET_RELAY Set a relay to a condition.
    * entry 182 DO_REPEAT_RELAY Cycle a relay on and off for a desired number of cyles with a desired period.
    * entry 183 DO_SET_SERVO Set a servo to a desired PWM value.
    * entry 184 DO_REPEAT_SERVO Cycle a between its nominal setting and a desired PWM for a desired number of cycles with a desired period.
    * entry 185 DO_FLIGHTTERMINATION Terminate flight immediately
    * entry 186 DO_CHANGE_ALTITUDE Change altitude set point.
    * entry 189 DO_LAND_START Mission command to perform a landing. This is used as a marker in a mission to tell the autopilot where a sequence of mission items that represents a landing starts. It may also be sent via a COMMAND_LONG to trigger a landing, in which case the nearest (geographically) landing sequence in the mission will be used. The Latitude/Longitude is optional, and may be set to 0 if not needed. If specified then it will be used to help find the closest landing sequence.
    * entry 190 DO_RALLY_LAND Mission command to perform a landing from a rally point.
    * entry 191 DO_GO_AROUND Mission command to safely abort an autonmous landing.
    * entry 192 DO_REPOSITION Reposition the vehicle to a specific WGS84 global position.
    * entry 193 DO_PAUSE_CONTINUE If in a GPS controlled position mode, hold the current position or continue.
    * entry 194 DO_SET_REVERSE Set moving direction to forward or reverse.
    * entry 200 DO_CONTROL_VIDEO Control onboard camera system.
    * entry 201 DO_SET_ROI Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras.
    * entry 202 DO_DIGICAM_CONFIGURE Mission command to configure an on-board camera controller system.
    * entry 203 DO_DIGICAM_CONTROL Mission command to control an on-board camera controller system.
    * entry 204 DO_MOUNT_CONFIGURE Mission command to configure a camera or antenna mount
    * entry 205 DO_MOUNT_CONTROL Mission command to control a camera or antenna mount
    * entry 206 DO_SET_CAM_TRIGG_DIST Mission command to set camera trigger distance for this flight. The camera is trigerred each time this distance is exceeded. This command can also be used to set the shutter integration time for the camera.
    * entry 207 DO_FENCE_ENABLE Mission command to enable the geofence
    * entry 208 DO_PARACHUTE Mission command to trigger a parachute
    * entry 209 DO_MOTOR_TEST Mission command to perform motor test
    * entry 210 DO_INVERTED_FLIGHT Change to/from inverted flight
    * entry 213 NAV_SET_YAW_SPEED Sets a desired vehicle turn angle and speed change
    * entry 214 DO_SET_CAM_TRIGG_INTERVAL Mission command to set camera trigger interval for this flight. If triggering is enabled, the camera is triggered each time this interval expires. This command can also be used to set the shutter integration time for the camera.
    * entry 220 DO_MOUNT_CONTROL_QUAT Mission command to control a camera or antenna mount, using a quaternion as reference.
    * entry 221 DO_GUIDED_MASTER set id of master controller
    * entry 222 DO_GUIDED_LIMITS set limits for external control
    * entry 223 DO_ENGINE_CONTROL Control vehicle engine. This is interpreted by the vehicles engine controller to change the target engine state. It is intended for vehicles with internal combustion engines
    * entry 240 DO_LAST NOP - This command is only used to mark the upper limit of the DO commands in the enumeration
    * entry 241 PREFLIGHT_CALIBRATION Trigger calibration. This command will be only accepted if in pre-flight mode. Except for Temperature Calibration, only one sensor should be set in a single message and all others should be zero.
    * entry 242 PREFLIGHT_SET_SENSOR_OFFSETS Set sensor offsets. This command will be only accepted if in pre-flight mode.
    * entry 243 PREFLIGHT_UAVCAN Trigger UAVCAN config. This command will be only accepted if in pre-flight mode.
    * entry 245 PREFLIGHT_STORAGE Request storage of different parameter values and logs. This command will be only accepted if in pre-flight mode.
    * entry 246 PREFLIGHT_REBOOT_SHUTDOWN Request the reboot or shutdown of system components.
    * entry 252 OVERRIDE_GOTO Hold / continue the current action
    * entry 300 MISSION_START start running a mission
    * entry 400 COMPONENT_ARM_DISARM Arms / Disarms a component
    * entry 410 GET_HOME_POSITION Request the home position from the vehicle.
    * entry 500 START_RX_PAIR Starts receiver pairing
    * entry 510 GET_MESSAGE_INTERVAL Request the interval between messages for a particular MAVLink message ID
    * entry 511 SET_MESSAGE_INTERVAL Request the interval between messages for a particular MAVLink message ID. This interface replaces REQUEST_DATA_STREAM
    * entry 519 REQUEST_PROTOCOL_VERSION Request MAVLink protocol version compatibility
    * entry 520 REQUEST_AUTOPILOT_CAPABILITIES Request autopilot capabilities
    * entry 521 REQUEST_CAMERA_INFORMATION WIP: Request camera information (CAMERA_INFORMATION).
    * entry 522 REQUEST_CAMERA_SETTINGS WIP: Request camera settings (CAMERA_SETTINGS).
    * entry 525 REQUEST_STORAGE_INFORMATION WIP: Request storage information (STORAGE_INFORMATION). Use the command's target_component to target a specific component's storage.
    * entry 526 STORAGE_FORMAT WIP: Format a storage medium. Once format is complete, a STORAGE_INFORMATION message is sent. Use the command's target_component to target a specific component's storage.
    * entry 527 REQUEST_CAMERA_CAPTURE_STATUS WIP: Request camera capture status (CAMERA_CAPTURE_STATUS)
    * entry 528 REQUEST_FLIGHT_INFORMATION WIP: Request flight information (FLIGHT_INFORMATION)
    * entry 529 RESET_CAMERA_SETTINGS WIP: Reset all camera settings to Factory Default
    * entry 530 SET_CAMERA_MODE Set camera running mode. Use NAN for reserved values.
    * entry 2000 IMAGE_START_CAPTURE Start image capture sequence. Sends CAMERA_IMAGE_CAPTURED after each capture. Use NAN for reserved values.
    * entry 2001 IMAGE_STOP_CAPTURE Stop image capture sequence Use NAN for reserved values.
    * entry 2002 REQUEST_CAMERA_IMAGE_CAPTURE WIP: Re-request a CAMERA_IMAGE_CAPTURE packet. Use NAN for reserved values.
    * entry 2003 DO_TRIGGER_CONTROL Enable or disable on-board camera triggering system.
    * entry 2500 VIDEO_START_CAPTURE Starts video capture (recording). Use NAN for reserved values.
    * entry 2501 VIDEO_STOP_CAPTURE Stop the current video capture (recording). Use NAN for reserved values.
    * entry 2502 VIDEO_START_STREAMING WIP: Start video streaming
    * entry 2503 VIDEO_STOP_STREAMING WIP: Stop the current video streaming
    * entry 2504 REQUEST_VIDEO_STREAM_INFORMATION WIP: Request video stream information (VIDEO_STREAM_INFORMATION)
    * entry 2510 LOGGING_START Request to start streaming logging data over MAVLink (see also LOGGING_DATA message)
    * entry 2511 LOGGING_STOP Request to stop streaming log data over MAVLink
    * entry 2520 AIRFRAME_CONFIGURATION
    * entry 2800 PANORAMA_CREATE Create a panorama at the current position
    * entry 3000 DO_VTOL_TRANSITION Request VTOL transition
    * entry 4000 SET_GUIDED_SUBMODE_STANDARD This command sets the submode to standard guided when vehicle is in guided mode. The vehicle holds position and altitude and the user can input the desired velocites along all three axes.
    *
    * entry 4001 SET_GUIDED_SUBMODE_CIRCLE This command sets submode circle when vehicle is in guided mode. Vehicle flies along a circle facing the center of the circle. The user can input the velocity along the circle and change the radius. If no input is given the vehicle will hold position.
    *
    * entry 4501 CONDITION_GATE WIP: Delay mission state machine until gate has been reached.
    * entry 3001 ARM_AUTHORIZATION_REQUEST Request authorization to arm the vehicle to a external entity, the arm authorizer is resposible to request all data that is needs from the vehicle before authorize or deny the request. If approved the progress of command_ack message should be set with period of time that this authorization is valid in seconds or in case it was denied it should be set with one of the reasons in ARM_AUTH_DENIED_REASON.
    *
    * entry 5000 NAV_FENCE_RETURN_POINT Fence return point. There can only be one fence return point.
    *
    * entry 5001 NAV_FENCE_POLYGON_VERTEX_INCLUSION Fence vertex for an inclusion polygon (the polygon must not be self-intersecting). The vehicle must stay within this area. Minimum of 3 vertices required.
    *
    * entry 5002 NAV_FENCE_POLYGON_VERTEX_EXCLUSION Fence vertex for an exclusion polygon (the polygon must not be self-intersecting). The vehicle must stay outside this area. Minimum of 3 vertices required.
    *
    * entry 5003 NAV_FENCE_CIRCLE_INCLUSION Circular fence area. The vehicle must stay inside this area.
    *
    * entry 5004 NAV_FENCE_CIRCLE_EXCLUSION Circular fence area. The vehicle must stay outside this area.
    *
    * entry 5100 NAV_RALLY_POINT Rally point. You can have multiple rally points defined.
    *
    * entry 5200 UAVCAN_GET_NODE_INFO Commands the vehicle to respond with a sequence of messages UAVCAN_NODE_INFO, one message per every UAVCAN node that is online. Note that some of the response messages can be lost, which the receiver can detect easily by checking whether every received UAVCAN_NODE_STATUS has a matching message UAVCAN_NODE_INFO received earlier; if not, this command should be sent again in order to request re-transmission of the node information messages.
    * entry 30001 PAYLOAD_PREPARE_DEPLOY Deploy payload on a Lat / Lon / Alt position. This includes the navigation to reach the required release position and velocity.
    * entry 30002 PAYLOAD_CONTROL_DEPLOY Control the payload deployment.
    * entry 31000 WAYPOINT_USER_1 User defined waypoint item. Ground Station will show the Vehicle as flying through this item.
    * entry 31001 WAYPOINT_USER_2 User defined waypoint item. Ground Station will show the Vehicle as flying through this item.
    * entry 31002 WAYPOINT_USER_3 User defined waypoint item. Ground Station will show the Vehicle as flying through this item.
    * entry 31003 WAYPOINT_USER_4 User defined waypoint item. Ground Station will show the Vehicle as flying through this item.
    * entry 31004 WAYPOINT_USER_5 User defined waypoint item. Ground Station will show the Vehicle as flying through this item.
    * entry 31005 SPATIAL_USER_1 User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item.
    * entry 31006 SPATIAL_USER_2 User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item.
    * entry 31007 SPATIAL_USER_3 User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item.
    * entry 31008 SPATIAL_USER_4 User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item.
    * entry 31009 SPATIAL_USER_5 User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item.
    * entry 31010 USER_1 User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item.
    * entry 31011 USER_2 User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item.
    * entry 31012 USER_3 User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item.
    * entry 31013 USER_4 User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item.
    * entry 31014 USER_5 User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item.
    * }}}
    */
  class MAVCmd(val value: Int)

  object MAVCmd {

    case object NavWaypoint extends MAVCmd(16)

    case object NavLoiterUnlim extends MAVCmd(17)

    case object NavLoiterTurns extends MAVCmd(18)

    case object NavLoiterTime extends MAVCmd(19)

    case object NavReturnToLaunch extends MAVCmd(20)

    case object NavLand extends MAVCmd(21)

    case object NavTakeoff extends MAVCmd(22)

    case object NavLandLocal extends MAVCmd(23)

    case object NavTakeoffLocal extends MAVCmd(24)

    case object NavFollow extends MAVCmd(25)

    case object NavContinueAndChangeAlt extends MAVCmd(30)

    case object NavLoiterToAlt extends MAVCmd(31)

    case object DoFollow extends MAVCmd(32)

    case object DoFollowReposition extends MAVCmd(33)

    case object NavROI extends MAVCmd(80)

    case object NavPathplanning extends MAVCmd(81)

    case object NavSplineWaypoint extends MAVCmd(82)

    case object NavVTOLTakeoff extends MAVCmd(84)

    case object NavVTOLLand extends MAVCmd(85)

    case object NavGuidedEnable extends MAVCmd(92)

    case object NavDelay extends MAVCmd(93)

    case object NavPayloadPlace extends MAVCmd(94)

    case object NavLast extends MAVCmd(95)

    case object ConditionDelay extends MAVCmd(112)

    case object ConditionChangeAlt extends MAVCmd(113)

    case object ConditionDistance extends MAVCmd(114)

    case object ConditionYaw extends MAVCmd(115)

    case object ConditionLast extends MAVCmd(159)

    case object DoSetMode extends MAVCmd(176)

    case object DoJump extends MAVCmd(177)

    case object DoChangeSpeed extends MAVCmd(178)

    case object DoSetHome extends MAVCmd(179)

    case object DoSetParameter extends MAVCmd(180)

    case object DoSetRelay extends MAVCmd(181)

    case object DoRepeatRelay extends MAVCmd(182)

    case object DoSetServo extends MAVCmd(183)

    case object DoRepeatServo extends MAVCmd(184)

    case object DoFlighttermination extends MAVCmd(185)

    case object DoChangeAltitude extends MAVCmd(186)

    case object DoLandStart extends MAVCmd(189)

    case object DoRallyLand extends MAVCmd(190)

    case object DoGoAround extends MAVCmd(191)

    case object DoReposition extends MAVCmd(192)

    case object DoPauseContinue extends MAVCmd(193)

    case object DoSetReverse extends MAVCmd(194)

    case object DoControlVideo extends MAVCmd(200)

    case object DoSetROI extends MAVCmd(201)

    case object DoDigicamConfigure extends MAVCmd(202)

    case object DoDigicamControl extends MAVCmd(203)

    case object DoMountConfigure extends MAVCmd(204)

    case object DoMountControl extends MAVCmd(205)

    case object DoSetCamTriggDist extends MAVCmd(206)

    case object DoFenceEnable extends MAVCmd(207)

    case object DoParachute extends MAVCmd(208)

    case object DoMotorTest extends MAVCmd(209)

    case object DoInvertedFlight extends MAVCmd(210)

    case object NavSetYawSpeed extends MAVCmd(213)

    case object DoSetCamTriggInterval extends MAVCmd(214)

    case object DoMountControlQuat extends MAVCmd(220)

    case object DoGuidedMaster extends MAVCmd(221)

    case object DoGuidedLimits extends MAVCmd(222)

    case object DoEngineControl extends MAVCmd(223)

    case object DoLast extends MAVCmd(240)

    case object PreflightCalibration extends MAVCmd(241)

    case object PreflightSetSensorOffsets extends MAVCmd(242)

    case object PreflightUAVCAN extends MAVCmd(243)

    case object PreflightStorage extends MAVCmd(245)

    case object PreflightRebootShutdown extends MAVCmd(246)

    case object OverrideGoto extends MAVCmd(252)

    case object MissionStart extends MAVCmd(300)

    case object ComponentArmDisarm extends MAVCmd(400)

    case object GetHomePosition extends MAVCmd(410)

    case object StartRXPair extends MAVCmd(500)

    case object GetMessageInterval extends MAVCmd(510)

    case object SetMessageInterval extends MAVCmd(511)

    case object RequestProtocolVersion extends MAVCmd(519)

    case object RequestAutopilotCapabilities extends MAVCmd(520)

    case object RequestCameraInformation extends MAVCmd(521)

    case object RequestCameraSettings extends MAVCmd(522)

    case object RequestStorageInformation extends MAVCmd(525)

    case object StorageFormat extends MAVCmd(526)

    case object RequestCameraCaptureStatus extends MAVCmd(527)

    case object RequestFlightInformation extends MAVCmd(528)

    case object ResetCameraSettings extends MAVCmd(529)

    case object SetCameraMode extends MAVCmd(530)

    case object ImageStartCapture extends MAVCmd(2000)

    case object ImageStopCapture extends MAVCmd(2001)

    case object RequestCameraImageCapture extends MAVCmd(2002)

    case object DoTriggerControl extends MAVCmd(2003)

    case object VideoStartCapture extends MAVCmd(2500)

    case object VideoStopCapture extends MAVCmd(2501)

    case object VideoStartStreaming extends MAVCmd(2502)

    case object VideoStopStreaming extends MAVCmd(2503)

    case object RequestVideoStreamInformation extends MAVCmd(2504)

    case object LoggingStart extends MAVCmd(2510)

    case object LoggingStop extends MAVCmd(2511)

    case object AirframeConfiguration extends MAVCmd(2520)

    case object PanoramaCreate extends MAVCmd(2800)

    case object DoVTOLTransition extends MAVCmd(3000)

    case object SetGuidedSubmodeStandard extends MAVCmd(4000)

    case object SetGuidedSubmodeCircle extends MAVCmd(4001)

    case object ConditionGate extends MAVCmd(4501)

    case object ArmAuthorizationRequest extends MAVCmd(3001)

    case object NavFenceReturnPoint extends MAVCmd(5000)

    case object NavFencePolygonVertexInclusion extends MAVCmd(5001)

    case object NavFencePolygonVertexExclusion extends MAVCmd(5002)

    case object NavFenceCircleInclusion extends MAVCmd(5003)

    case object NavFenceCircleExclusion extends MAVCmd(5004)

    case object NavRallyPoint extends MAVCmd(5100)

    case object UAVCANGetNodeInfo extends MAVCmd(5200)

    case object PayloadPrepareDeploy extends MAVCmd(30001)

    case object PayloadControlDeploy extends MAVCmd(30002)

    case object WaypointUser1 extends MAVCmd(31000)

    case object WaypointUser2 extends MAVCmd(31001)

    case object WaypointUser3 extends MAVCmd(31002)

    case object WaypointUser4 extends MAVCmd(31003)

    case object WaypointUser5 extends MAVCmd(31004)

    case object SpatialUser1 extends MAVCmd(31005)

    case object SpatialUser2 extends MAVCmd(31006)

    case object SpatialUser3 extends MAVCmd(31007)

    case object SpatialUser4 extends MAVCmd(31008)

    case object SpatialUser5 extends MAVCmd(31009)

    case object User1 extends MAVCmd(31010)

    case object User2 extends MAVCmd(31011)

    case object User3 extends MAVCmd(31012)

    case object User4 extends MAVCmd(31013)

    case object User5 extends MAVCmd(31014)

    val set: Set[MAVCmd] =
      Set(
        NavWaypoint,
        NavLoiterUnlim,
        NavLoiterTurns,
        NavLoiterTime,
        NavReturnToLaunch,
        NavLand,
        NavTakeoff,
        NavLandLocal,
        NavTakeoffLocal,
        NavFollow,
        NavContinueAndChangeAlt,
        NavLoiterToAlt,
        DoFollow,
        DoFollowReposition,
        NavROI,
        NavPathplanning,
        NavSplineWaypoint,
        NavVTOLTakeoff,
        NavVTOLLand,
        NavGuidedEnable,
        NavDelay,
        NavPayloadPlace,
        NavLast,
        ConditionDelay,
        ConditionChangeAlt,
        ConditionDistance,
        ConditionYaw,
        ConditionLast,
        DoSetMode,
        DoJump,
        DoChangeSpeed,
        DoSetHome,
        DoSetParameter,
        DoSetRelay,
        DoRepeatRelay,
        DoSetServo,
        DoRepeatServo,
        DoFlighttermination,
        DoChangeAltitude,
        DoLandStart,
        DoRallyLand,
        DoGoAround,
        DoReposition,
        DoPauseContinue,
        DoSetReverse,
        DoControlVideo,
        DoSetROI,
        DoDigicamConfigure,
        DoDigicamControl,
        DoMountConfigure,
        DoMountControl,
        DoSetCamTriggDist,
        DoFenceEnable,
        DoParachute,
        DoMotorTest,
        DoInvertedFlight,
        NavSetYawSpeed,
        DoSetCamTriggInterval,
        DoMountControlQuat,
        DoGuidedMaster,
        DoGuidedLimits,
        DoEngineControl,
        DoLast,
        PreflightCalibration,
        PreflightSetSensorOffsets,
        PreflightUAVCAN,
        PreflightStorage,
        PreflightRebootShutdown,
        OverrideGoto,
        MissionStart,
        ComponentArmDisarm,
        GetHomePosition,
        StartRXPair,
        GetMessageInterval,
        SetMessageInterval,
        RequestProtocolVersion,
        RequestAutopilotCapabilities,
        RequestCameraInformation,
        RequestCameraSettings,
        RequestStorageInformation,
        StorageFormat,
        RequestCameraCaptureStatus,
        RequestFlightInformation,
        ResetCameraSettings,
        SetCameraMode,
        ImageStartCapture,
        ImageStopCapture,
        RequestCameraImageCapture,
        DoTriggerControl,
        VideoStartCapture,
        VideoStopCapture,
        VideoStartStreaming,
        VideoStopStreaming,
        RequestVideoStreamInformation,
        LoggingStart,
        LoggingStop,
        AirframeConfiguration,
        PanoramaCreate,
        DoVTOLTransition,
        SetGuidedSubmodeStandard,
        SetGuidedSubmodeCircle,
        ConditionGate,
        ArmAuthorizationRequest,
        NavFenceReturnPoint,
        NavFencePolygonVertexInclusion,
        NavFencePolygonVertexExclusion,
        NavFenceCircleInclusion,
        NavFenceCircleExclusion,
        NavRallyPoint,
        UAVCANGetNodeInfo,
        PayloadPrepareDeploy,
        PayloadControlDeploy,
        WaypointUser1,
        WaypointUser2,
        WaypointUser3,
        WaypointUser4,
        WaypointUser5,
        SpatialUser1,
        SpatialUser2,
        SpatialUser3,
        SpatialUser4,
        SpatialUser5,
        User1,
        User2,
        User3,
        User4,
        User5
      )

    val map: Map[Int, MAVCmd] = set.toSeq.map(instance => instance.value -> instance).toMap

    def apply(value: Int): MAVCmd = map.getOrElse(value, new MAVCmd(value))

    def bitmask(value: Int): Set[MAVCmd] = {
      val known = set.filter(instance => (instance.value & value) == instance.value)
      val sum = known.map(_.value).sum
      if (value == sum) known else known + new MAVCmd(value - sum)
    }
  }

  /**
    * MAV_DATA_STREAM
    *
    * THIS INTERFACE IS DEPRECATED AS OF JULY 2015. Please use MESSAGE_INTERVAL instead. A data stream is not a fixed set of messages, but rather a
    * recommendation to the autopilot software. Individual autopilots may or may not obey
    * the recommended messages.
    *
    * {{{
    * entry 0 ALL Enable all data streams
    * entry 1 RAW_SENSORS Enable IMU_RAW, GPS_RAW, GPS_STATUS packets.
    * entry 2 EXTENDED_STATUS Enable GPS_STATUS, CONTROL_STATUS, AUX_STATUS
    * entry 3 RC_CHANNELS Enable RC_CHANNELS_SCALED, RC_CHANNELS_RAW, SERVO_OUTPUT_RAW
    * entry 4 RAW_CONTROLLER Enable ATTITUDE_CONTROLLER_OUTPUT, POSITION_CONTROLLER_OUTPUT, NAV_CONTROLLER_OUTPUT.
    * entry 6 POSITION Enable LOCAL_POSITION, GLOBAL_POSITION/GLOBAL_POSITION_INT messages.
    * entry 10 EXTRA1 Dependent on the autopilot
    * entry 11 EXTRA2 Dependent on the autopilot
    * entry 12 EXTRA3 Dependent on the autopilot
    * }}}
    */
  class MAVDataStream(val value: Int)

  object MAVDataStream {

    case object All extends MAVDataStream(0)

    case object RawSensors extends MAVDataStream(1)

    case object ExtendedStatus extends MAVDataStream(2)

    case object RCChannels extends MAVDataStream(3)

    case object RawController extends MAVDataStream(4)

    case object Position extends MAVDataStream(6)

    case object Extra1 extends MAVDataStream(10)

    case object Extra2 extends MAVDataStream(11)

    case object Extra3 extends MAVDataStream(12)

    val set: Set[MAVDataStream] =
      Set(
        All,
        RawSensors,
        ExtendedStatus,
        RCChannels,
        RawController,
        Position,
        Extra1,
        Extra2,
        Extra3
      )

    val map: Map[Int, MAVDataStream] = set.toSeq.map(instance => instance.value -> instance).toMap

    def apply(value: Int): MAVDataStream = map.getOrElse(value, new MAVDataStream(value))

    def bitmask(value: Int): Set[MAVDataStream] = {
      val known = set.filter(instance => (instance.value & value) == instance.value)
      val sum = known.map(_.value).sum
      if (value == sum) known else known + new MAVDataStream(value - sum)
    }
  }

  /**
    * MAV_ROI
    *
    * The ROI (region of interest) for the vehicle. This can be
    * be used by the vehicle for camera/vehicle attitude alignment (see
    * MAV_CMD_NAV_ROI).
    *
    * {{{
    * entry 0 NONE No region of interest.
    * entry 1 WPNEXT Point toward next waypoint.
    * entry 2 WPINDEX Point toward given waypoint.
    * entry 3 LOCATION Point toward fixed location.
    * entry 4 TARGET Point toward of given id.
    * }}}
    */
  class MAVROI(val value: Int)

  object MAVROI {

    case object None extends MAVROI(0)

    case object WPNext extends MAVROI(1)

    case object WPIndex extends MAVROI(2)

    case object Location extends MAVROI(3)

    case object Target extends MAVROI(4)

    val set: Set[MAVROI] =
      Set(
        None,
        WPNext,
        WPIndex,
        Location,
        Target
      )

    val map: Map[Int, MAVROI] = set.toSeq.map(instance => instance.value -> instance).toMap

    def apply(value: Int): MAVROI = map.getOrElse(value, new MAVROI(value))

    def bitmask(value: Int): Set[MAVROI] = {
      val known = set.filter(instance => (instance.value & value) == instance.value)
      val sum = known.map(_.value).sum
      if (value == sum) known else known + new MAVROI(value - sum)
    }
  }

  /**
    * MAV_CMD_ACK
    *
    * ACK / NACK / ERROR values as a result of MAV_CMDs and for mission item transmission.
    *
    * {{{
    * entry 1 OK Command / mission item is ok.
    * entry 2 ERR_FAIL Generic error message if none of the other reasons fails or if no detailed error reporting is implemented.
    * entry 3 ERR_ACCESS_DENIED The system is refusing to accept this command from this source / communication partner.
    * entry 4 ERR_NOT_SUPPORTED Command or mission item is not supported, other commands would be accepted.
    * entry 5 ERR_COORDINATE_FRAME_NOT_SUPPORTED The coordinate frame of this command / mission item is not supported.
    * entry 6 ERR_COORDINATES_OUT_OF_RANGE The coordinate frame of this command is ok, but he coordinate values exceed the safety limits of this system. This is a generic error, please use the more specific error messages below if possible.
    * entry 7 ERR_X_LAT_OUT_OF_RANGE The X or latitude value is out of range.
    * entry 8 ERR_Y_LON_OUT_OF_RANGE The Y or longitude value is out of range.
    * entry 9 ERR_Z_ALT_OUT_OF_RANGE The Z or altitude value is out of range.
    * }}}
    */
  class MAVCmdAck(val value: Int)

  object MAVCmdAck {

    case object OK extends MAVCmdAck(1)

    case object ErrFail extends MAVCmdAck(2)

    case object ErrAccessDenied extends MAVCmdAck(3)

    case object ErrNotSupported extends MAVCmdAck(4)

    case object ErrCoordinateFrameNotSupported extends MAVCmdAck(5)

    case object ErrCoordinatesOutOfRange extends MAVCmdAck(6)

    case object ErrXLatOutOfRange extends MAVCmdAck(7)

    case object ErrYLonOutOfRange extends MAVCmdAck(8)

    case object ErrZAltOutOfRange extends MAVCmdAck(9)

    val set: Set[MAVCmdAck] =
      Set(
        OK,
        ErrFail,
        ErrAccessDenied,
        ErrNotSupported,
        ErrCoordinateFrameNotSupported,
        ErrCoordinatesOutOfRange,
        ErrXLatOutOfRange,
        ErrYLonOutOfRange,
        ErrZAltOutOfRange
      )

    val map: Map[Int, MAVCmdAck] = set.toSeq.map(instance => instance.value -> instance).toMap

    def apply(value: Int): MAVCmdAck = map.getOrElse(value, new MAVCmdAck(value))

    def bitmask(value: Int): Set[MAVCmdAck] = {
      val known = set.filter(instance => (instance.value & value) == instance.value)
      val sum = known.map(_.value).sum
      if (value == sum) known else known + new MAVCmdAck(value - sum)
    }
  }

  /**
    * MAV_PARAM_TYPE
    *
    * Specifies the datatype of a MAVLink parameter.
    *
    * {{{
    * entry 1 UINT8 8-bit unsigned integer
    * entry 2 INT8 8-bit signed integer
    * entry 3 UINT16 16-bit unsigned integer
    * entry 4 INT16 16-bit signed integer
    * entry 5 UINT32 32-bit unsigned integer
    * entry 6 INT32 32-bit signed integer
    * entry 7 UINT64 64-bit unsigned integer
    * entry 8 INT64 64-bit signed integer
    * entry 9 REAL32 32-bit floating-point
    * entry 10 REAL64 64-bit floating-point
    * }}}
    */
  class MAVParamType(val value: Int)

  object MAVParamType {

    case object Uint8 extends MAVParamType(1)

    case object Int8 extends MAVParamType(2)

    case object Uint16 extends MAVParamType(3)

    case object Int16 extends MAVParamType(4)

    case object Uint32 extends MAVParamType(5)

    case object Int32 extends MAVParamType(6)

    case object Uint64 extends MAVParamType(7)

    case object Int64 extends MAVParamType(8)

    case object Real32 extends MAVParamType(9)

    case object Real64 extends MAVParamType(10)

    val set: Set[MAVParamType] =
      Set(
        Uint8,
        Int8,
        Uint16,
        Int16,
        Uint32,
        Int32,
        Uint64,
        Int64,
        Real32,
        Real64
      )

    val map: Map[Int, MAVParamType] = set.toSeq.map(instance => instance.value -> instance).toMap

    def apply(value: Int): MAVParamType = map.getOrElse(value, new MAVParamType(value))

    def bitmask(value: Int): Set[MAVParamType] = {
      val known = set.filter(instance => (instance.value & value) == instance.value)
      val sum = known.map(_.value).sum
      if (value == sum) known else known + new MAVParamType(value - sum)
    }
  }

  /**
    * MAV_PARAM_EXT_TYPE
    *
    * Specifies the datatype of a MAVLink extended parameter.
    *
    * {{{
    * entry 1 UINT8 8-bit unsigned integer
    * entry 2 INT8 8-bit signed integer
    * entry 3 UINT16 16-bit unsigned integer
    * entry 4 INT16 16-bit signed integer
    * entry 5 UINT32 32-bit unsigned integer
    * entry 6 INT32 32-bit signed integer
    * entry 7 UINT64 64-bit unsigned integer
    * entry 8 INT64 64-bit signed integer
    * entry 9 REAL32 32-bit floating-point
    * entry 10 REAL64 64-bit floating-point
    * entry 11 CUSTOM Custom Type
    * }}}
    */
  class MAVParamExtType(val value: Int)

  object MAVParamExtType {

    case object Uint8 extends MAVParamExtType(1)

    case object Int8 extends MAVParamExtType(2)

    case object Uint16 extends MAVParamExtType(3)

    case object Int16 extends MAVParamExtType(4)

    case object Uint32 extends MAVParamExtType(5)

    case object Int32 extends MAVParamExtType(6)

    case object Uint64 extends MAVParamExtType(7)

    case object Int64 extends MAVParamExtType(8)

    case object Real32 extends MAVParamExtType(9)

    case object Real64 extends MAVParamExtType(10)

    case object Custom extends MAVParamExtType(11)

    val set: Set[MAVParamExtType] =
      Set(
        Uint8,
        Int8,
        Uint16,
        Int16,
        Uint32,
        Int32,
        Uint64,
        Int64,
        Real32,
        Real64,
        Custom
      )

    val map: Map[Int, MAVParamExtType] = set.toSeq.map(instance => instance.value -> instance).toMap

    def apply(value: Int): MAVParamExtType = map.getOrElse(value, new MAVParamExtType(value))

    def bitmask(value: Int): Set[MAVParamExtType] = {
      val known = set.filter(instance => (instance.value & value) == instance.value)
      val sum = known.map(_.value).sum
      if (value == sum) known else known + new MAVParamExtType(value - sum)
    }
  }

  /**
    * MAV_RESULT
    *
    * result from a mavlink command
    *
    * {{{
    * entry 0 ACCEPTED Command ACCEPTED and EXECUTED
    * entry 1 TEMPORARILY_REJECTED Command TEMPORARY REJECTED/DENIED
    * entry 2 DENIED Command PERMANENTLY DENIED
    * entry 3 UNSUPPORTED Command UNKNOWN/UNSUPPORTED
    * entry 4 FAILED Command executed, but failed
    * entry 5 IN_PROGRESS WIP: Command being executed
    * }}}
    */
  class MAVResult(val value: Int)

  object MAVResult {

    case object Accepted extends MAVResult(0)

    case object TemporarilyRejected extends MAVResult(1)

    case object Denied extends MAVResult(2)

    case object Unsupported extends MAVResult(3)

    case object Failed extends MAVResult(4)

    case object InProgress extends MAVResult(5)

    val set: Set[MAVResult] =
      Set(
        Accepted,
        TemporarilyRejected,
        Denied,
        Unsupported,
        Failed,
        InProgress
      )

    val map: Map[Int, MAVResult] = set.toSeq.map(instance => instance.value -> instance).toMap

    def apply(value: Int): MAVResult = map.getOrElse(value, new MAVResult(value))

    def bitmask(value: Int): Set[MAVResult] = {
      val known = set.filter(instance => (instance.value & value) == instance.value)
      val sum = known.map(_.value).sum
      if (value == sum) known else known + new MAVResult(value - sum)
    }
  }

  /**
    * MAV_MISSION_RESULT
    *
    * result in a mavlink mission ack
    *
    * {{{
    * entry 0 ACCEPTED mission accepted OK
    * entry 1 ERROR generic error / not accepting mission commands at all right now
    * entry 2 UNSUPPORTED_FRAME coordinate frame is not supported
    * entry 3 UNSUPPORTED command is not supported
    * entry 4 NO_SPACE mission item exceeds storage space
    * entry 5 INVALID one of the parameters has an invalid value
    * entry 6 INVALID_PARAM1 param1 has an invalid value
    * entry 7 INVALID_PARAM2 param2 has an invalid value
    * entry 8 INVALID_PARAM3 param3 has an invalid value
    * entry 9 INVALID_PARAM4 param4 has an invalid value
    * entry 10 INVALID_PARAM5_X x/param5 has an invalid value
    * entry 11 INVALID_PARAM6_Y y/param6 has an invalid value
    * entry 12 INVALID_PARAM7 param7 has an invalid value
    * entry 13 INVALID_SEQUENCE received waypoint out of sequence
    * entry 14 DENIED not accepting any mission commands from this communication partner
    * }}}
    */
  class MAVMissionResult(val value: Int)

  object MAVMissionResult {

    case object Accepted extends MAVMissionResult(0)

    case object Error extends MAVMissionResult(1)

    case object UnsupportedFrame extends MAVMissionResult(2)

    case object Unsupported extends MAVMissionResult(3)

    case object NoSpace extends MAVMissionResult(4)

    case object Invalid extends MAVMissionResult(5)

    case object InvalidParam1 extends MAVMissionResult(6)

    case object InvalidParam2 extends MAVMissionResult(7)

    case object InvalidParam3 extends MAVMissionResult(8)

    case object InvalidParam4 extends MAVMissionResult(9)

    case object InvalidParam5X extends MAVMissionResult(10)

    case object InvalidParam6Y extends MAVMissionResult(11)

    case object InvalidParam7 extends MAVMissionResult(12)

    case object InvalidSequence extends MAVMissionResult(13)

    case object Denied extends MAVMissionResult(14)

    val set: Set[MAVMissionResult] =
      Set(
        Accepted,
        Error,
        UnsupportedFrame,
        Unsupported,
        NoSpace,
        Invalid,
        InvalidParam1,
        InvalidParam2,
        InvalidParam3,
        InvalidParam4,
        InvalidParam5X,
        InvalidParam6Y,
        InvalidParam7,
        InvalidSequence,
        Denied
      )

    val map: Map[Int, MAVMissionResult] = set.toSeq.map(instance => instance.value -> instance).toMap

    def apply(value: Int): MAVMissionResult = map.getOrElse(value, new MAVMissionResult(value))

    def bitmask(value: Int): Set[MAVMissionResult] = {
      val known = set.filter(instance => (instance.value & value) == instance.value)
      val sum = known.map(_.value).sum
      if (value == sum) known else known + new MAVMissionResult(value - sum)
    }
  }

  /**
    * MAV_SEVERITY
    *
    * Indicates the severity level, generally used for status messages to indicate their relative urgency. Based on RFC-5424 using expanded definitions at: http://www.kiwisyslog.com/kb/info:-syslog-message-levels/.
    *
    * {{{
    * entry 0 EMERGENCY System is unusable. This is a "panic" condition.
    * entry 1 ALERT Action should be taken immediately. Indicates error in non-critical systems.
    * entry 2 CRITICAL Action must be taken immediately. Indicates failure in a primary system.
    * entry 3 ERROR Indicates an error in secondary/redundant systems.
    * entry 4 WARNING Indicates about a possible future error if this is not resolved within a given timeframe. Example would be a low battery warning.
    * entry 5 NOTICE An unusual event has occured, though not an error condition. This should be investigated for the root cause.
    * entry 6 INFO Normal operational messages. Useful for logging. No action is required for these messages.
    * entry 7 DEBUG Useful non-operational messages that can assist in debugging. These should not occur during normal operation.
    * }}}
    */
  class MAVSeverity(val value: Int)

  object MAVSeverity {

    case object Emergency extends MAVSeverity(0)

    case object Alert extends MAVSeverity(1)

    case object Critical extends MAVSeverity(2)

    case object Error extends MAVSeverity(3)

    case object Warning extends MAVSeverity(4)

    case object Notice extends MAVSeverity(5)

    case object Info extends MAVSeverity(6)

    case object Debug extends MAVSeverity(7)

    val set: Set[MAVSeverity] =
      Set(
        Emergency,
        Alert,
        Critical,
        Error,
        Warning,
        Notice,
        Info,
        Debug
      )

    val map: Map[Int, MAVSeverity] = set.toSeq.map(instance => instance.value -> instance).toMap

    def apply(value: Int): MAVSeverity = map.getOrElse(value, new MAVSeverity(value))

    def bitmask(value: Int): Set[MAVSeverity] = {
      val known = set.filter(instance => (instance.value & value) == instance.value)
      val sum = known.map(_.value).sum
      if (value == sum) known else known + new MAVSeverity(value - sum)
    }
  }

  /**
    * MAV_POWER_STATUS
    *
    * Power supply status flags (bitmask)
    *
    * {{{
    * entry 1 BRICK_VALID main brick power supply valid
    * entry 2 SERVO_VALID main servo power supply valid for FMU
    * entry 4 USB_CONNECTED USB power is connected
    * entry 8 PERIPH_OVERCURRENT peripheral supply is in over-current state
    * entry 16 PERIPH_HIPOWER_OVERCURRENT hi-power peripheral supply is in over-current state
    * entry 32 CHANGED Power status has changed since boot
    * }}}
    */
  class MAVPowerStatus(val value: Int)

  object MAVPowerStatus {

    case object BrickValid extends MAVPowerStatus(1)

    case object ServoValid extends MAVPowerStatus(2)

    case object USBConnected extends MAVPowerStatus(4)

    case object PeriphOvercurrent extends MAVPowerStatus(8)

    case object PeriphHipowerOvercurrent extends MAVPowerStatus(16)

    case object Changed extends MAVPowerStatus(32)

    val set: Set[MAVPowerStatus] =
      Set(
        BrickValid,
        ServoValid,
        USBConnected,
        PeriphOvercurrent,
        PeriphHipowerOvercurrent,
        Changed
      )

    val map: Map[Int, MAVPowerStatus] = set.toSeq.map(instance => instance.value -> instance).toMap

    def apply(value: Int): MAVPowerStatus = map.getOrElse(value, new MAVPowerStatus(value))

    def bitmask(value: Int): Set[MAVPowerStatus] = {
      val known = set.filter(instance => (instance.value & value) == instance.value)
      val sum = known.map(_.value).sum
      if (value == sum) known else known + new MAVPowerStatus(value - sum)
    }
  }

  /**
    * SERIAL_CONTROL_DEV
    *
    * SERIAL_CONTROL device types
    *
    * {{{
    * entry 0 TELEM1 First telemetry port
    * entry 1 TELEM2 Second telemetry port
    * entry 2 GPS1 First GPS port
    * entry 3 GPS2 Second GPS port
    * entry 10 SHELL system shell
    * }}}
    */
  class SerialControlDev(val value: Int)

  object SerialControlDev {

    case object Telem1 extends SerialControlDev(0)

    case object Telem2 extends SerialControlDev(1)

    case object Gps1 extends SerialControlDev(2)

    case object GPS2 extends SerialControlDev(3)

    case object Shell extends SerialControlDev(10)

    val set: Set[SerialControlDev] =
      Set(
        Telem1,
        Telem2,
        Gps1,
        GPS2,
        Shell
      )

    val map: Map[Int, SerialControlDev] = set.toSeq.map(instance => instance.value -> instance).toMap

    def apply(value: Int): SerialControlDev = map.getOrElse(value, new SerialControlDev(value))

    def bitmask(value: Int): Set[SerialControlDev] = {
      val known = set.filter(instance => (instance.value & value) == instance.value)
      val sum = known.map(_.value).sum
      if (value == sum) known else known + new SerialControlDev(value - sum)
    }
  }

  /**
    * SERIAL_CONTROL_FLAG
    *
    * SERIAL_CONTROL flags (bitmask)
    *
    * {{{
    * entry 1 REPLY Set if this is a reply
    * entry 2 RESPOND Set if the sender wants the receiver to send a response as another SERIAL_CONTROL message
    * entry 4 EXCLUSIVE Set if access to the serial port should be removed from whatever driver is currently using it, giving exclusive access to the SERIAL_CONTROL protocol. The port can be handed back by sending a request without this flag set
    * entry 8 BLOCKING Block on writes to the serial port
    * entry 16 MULTI Send multiple replies until port is drained
    * }}}
    */
  class SerialControlFlag(val value: Int)

  object SerialControlFlag {

    case object Reply extends SerialControlFlag(1)

    case object Respond extends SerialControlFlag(2)

    case object Exclusive extends SerialControlFlag(4)

    case object Blocking extends SerialControlFlag(8)

    case object Multi extends SerialControlFlag(16)

    val set: Set[SerialControlFlag] =
      Set(
        Reply,
        Respond,
        Exclusive,
        Blocking,
        Multi
      )

    val map: Map[Int, SerialControlFlag] = set.toSeq.map(instance => instance.value -> instance).toMap

    def apply(value: Int): SerialControlFlag = map.getOrElse(value, new SerialControlFlag(value))

    def bitmask(value: Int): Set[SerialControlFlag] = {
      val known = set.filter(instance => (instance.value & value) == instance.value)
      val sum = known.map(_.value).sum
      if (value == sum) known else known + new SerialControlFlag(value - sum)
    }
  }

  /**
    * MAV_DISTANCE_SENSOR
    *
    * Enumeration of distance sensor types
    *
    * {{{
    * entry 0 LASER Laser rangefinder, e.g. LightWare SF02/F or PulsedLight units
    * entry 1 ULTRASOUND Ultrasound rangefinder, e.g. MaxBotix units
    * entry 2 INFRARED Infrared rangefinder, e.g. Sharp units
    * entry 3 RADAR Radar type, e.g. uLanding units
    * entry 4 UNKNOWN Broken or unknown type, e.g. analog units
    * }}}
    */
  class MAVDistanceSensor(val value: Int)

  object MAVDistanceSensor {

    case object Laser extends MAVDistanceSensor(0)

    case object Ultrasound extends MAVDistanceSensor(1)

    case object Infrared extends MAVDistanceSensor(2)

    case object Radar extends MAVDistanceSensor(3)

    case object Unknown extends MAVDistanceSensor(4)

    val set: Set[MAVDistanceSensor] =
      Set(
        Laser,
        Ultrasound,
        Infrared,
        Radar,
        Unknown
      )

    val map: Map[Int, MAVDistanceSensor] = set.toSeq.map(instance => instance.value -> instance).toMap

    def apply(value: Int): MAVDistanceSensor = map.getOrElse(value, new MAVDistanceSensor(value))

    def bitmask(value: Int): Set[MAVDistanceSensor] = {
      val known = set.filter(instance => (instance.value & value) == instance.value)
      val sum = known.map(_.value).sum
      if (value == sum) known else known + new MAVDistanceSensor(value - sum)
    }
  }

  /**
    * MAV_SENSOR_ORIENTATION
    *
    * Enumeration of sensor orientation, according to its rotations
    *
    * {{{
    * entry 0 ROTATION_NONE Roll: 0, Pitch: 0, Yaw: 0
    * entry 1 ROTATION_YAW_45 Roll: 0, Pitch: 0, Yaw: 45
    * entry 2 ROTATION_YAW_90 Roll: 0, Pitch: 0, Yaw: 90
    * entry 3 ROTATION_YAW_135 Roll: 0, Pitch: 0, Yaw: 135
    * entry 4 ROTATION_YAW_180 Roll: 0, Pitch: 0, Yaw: 180
    * entry 5 ROTATION_YAW_225 Roll: 0, Pitch: 0, Yaw: 225
    * entry 6 ROTATION_YAW_270 Roll: 0, Pitch: 0, Yaw: 270
    * entry 7 ROTATION_YAW_315 Roll: 0, Pitch: 0, Yaw: 315
    * entry 8 ROTATION_ROLL_180 Roll: 180, Pitch: 0, Yaw: 0
    * entry 9 ROTATION_ROLL_180_YAW_45 Roll: 180, Pitch: 0, Yaw: 45
    * entry 10 ROTATION_ROLL_180_YAW_90 Roll: 180, Pitch: 0, Yaw: 90
    * entry 11 ROTATION_ROLL_180_YAW_135 Roll: 180, Pitch: 0, Yaw: 135
    * entry 12 ROTATION_PITCH_180 Roll: 0, Pitch: 180, Yaw: 0
    * entry 13 ROTATION_ROLL_180_YAW_225 Roll: 180, Pitch: 0, Yaw: 225
    * entry 14 ROTATION_ROLL_180_YAW_270 Roll: 180, Pitch: 0, Yaw: 270
    * entry 15 ROTATION_ROLL_180_YAW_315 Roll: 180, Pitch: 0, Yaw: 315
    * entry 16 ROTATION_ROLL_90 Roll: 90, Pitch: 0, Yaw: 0
    * entry 17 ROTATION_ROLL_90_YAW_45 Roll: 90, Pitch: 0, Yaw: 45
    * entry 18 ROTATION_ROLL_90_YAW_90 Roll: 90, Pitch: 0, Yaw: 90
    * entry 19 ROTATION_ROLL_90_YAW_135 Roll: 90, Pitch: 0, Yaw: 135
    * entry 20 ROTATION_ROLL_270 Roll: 270, Pitch: 0, Yaw: 0
    * entry 21 ROTATION_ROLL_270_YAW_45 Roll: 270, Pitch: 0, Yaw: 45
    * entry 22 ROTATION_ROLL_270_YAW_90 Roll: 270, Pitch: 0, Yaw: 90
    * entry 23 ROTATION_ROLL_270_YAW_135 Roll: 270, Pitch: 0, Yaw: 135
    * entry 24 ROTATION_PITCH_90 Roll: 0, Pitch: 90, Yaw: 0
    * entry 25 ROTATION_PITCH_270 Roll: 0, Pitch: 270, Yaw: 0
    * entry 26 ROTATION_PITCH_180_YAW_90 Roll: 0, Pitch: 180, Yaw: 90
    * entry 27 ROTATION_PITCH_180_YAW_270 Roll: 0, Pitch: 180, Yaw: 270
    * entry 28 ROTATION_ROLL_90_PITCH_90 Roll: 90, Pitch: 90, Yaw: 0
    * entry 29 ROTATION_ROLL_180_PITCH_90 Roll: 180, Pitch: 90, Yaw: 0
    * entry 30 ROTATION_ROLL_270_PITCH_90 Roll: 270, Pitch: 90, Yaw: 0
    * entry 31 ROTATION_ROLL_90_PITCH_180 Roll: 90, Pitch: 180, Yaw: 0
    * entry 32 ROTATION_ROLL_270_PITCH_180 Roll: 270, Pitch: 180, Yaw: 0
    * entry 33 ROTATION_ROLL_90_PITCH_270 Roll: 90, Pitch: 270, Yaw: 0
    * entry 34 ROTATION_ROLL_180_PITCH_270 Roll: 180, Pitch: 270, Yaw: 0
    * entry 35 ROTATION_ROLL_270_PITCH_270 Roll: 270, Pitch: 270, Yaw: 0
    * entry 36 ROTATION_ROLL_90_PITCH_180_YAW_90 Roll: 90, Pitch: 180, Yaw: 90
    * entry 37 ROTATION_ROLL_90_YAW_270 Roll: 90, Pitch: 0, Yaw: 270
    * entry 38 ROTATION_ROLL_315_PITCH_315_YAW_315 Roll: 315, Pitch: 315, Yaw: 315
    * }}}
    */
  class MAVSensorOrientation(val value: Int)

  object MAVSensorOrientation {

    case object RotationNone extends MAVSensorOrientation(0)

    case object RotationYaw45 extends MAVSensorOrientation(1)

    case object RotationYaw90 extends MAVSensorOrientation(2)

    case object RotationYaw135 extends MAVSensorOrientation(3)

    case object RotationYaw180 extends MAVSensorOrientation(4)

    case object RotationYaw225 extends MAVSensorOrientation(5)

    case object RotationYaw270 extends MAVSensorOrientation(6)

    case object RotationYaw315 extends MAVSensorOrientation(7)

    case object RotationRoll180 extends MAVSensorOrientation(8)

    case object RotationRoll180Yaw45 extends MAVSensorOrientation(9)

    case object RotationRoll180Yaw90 extends MAVSensorOrientation(10)

    case object RotationRoll180Yaw135 extends MAVSensorOrientation(11)

    case object RotationPitch180 extends MAVSensorOrientation(12)

    case object RotationRoll180Yaw225 extends MAVSensorOrientation(13)

    case object RotationRoll180Yaw270 extends MAVSensorOrientation(14)

    case object RotationRoll180Yaw315 extends MAVSensorOrientation(15)

    case object RotationRoll90 extends MAVSensorOrientation(16)

    case object RotationRoll90Yaw45 extends MAVSensorOrientation(17)

    case object RotationRoll90Yaw90 extends MAVSensorOrientation(18)

    case object RotationRoll90Yaw135 extends MAVSensorOrientation(19)

    case object RotationRoll270 extends MAVSensorOrientation(20)

    case object RotationRoll270Yaw45 extends MAVSensorOrientation(21)

    case object RotationRoll270Yaw90 extends MAVSensorOrientation(22)

    case object RotationRoll270Yaw135 extends MAVSensorOrientation(23)

    case object RotationPitch90 extends MAVSensorOrientation(24)

    case object RotationPitch270 extends MAVSensorOrientation(25)

    case object RotationPitch180Yaw90 extends MAVSensorOrientation(26)

    case object RotationPitch180Yaw270 extends MAVSensorOrientation(27)

    case object RotationRoll90Pitch90 extends MAVSensorOrientation(28)

    case object RotationRoll180Pitch90 extends MAVSensorOrientation(29)

    case object RotationRoll270Pitch90 extends MAVSensorOrientation(30)

    case object RotationRoll90Pitch180 extends MAVSensorOrientation(31)

    case object RotationRoll270Pitch180 extends MAVSensorOrientation(32)

    case object RotationRoll90Pitch270 extends MAVSensorOrientation(33)

    case object RotationRoll180Pitch270 extends MAVSensorOrientation(34)

    case object RotationRoll270Pitch270 extends MAVSensorOrientation(35)

    case object RotationRoll90Pitch180Yaw90 extends MAVSensorOrientation(36)

    case object RotationRoll90Yaw270 extends MAVSensorOrientation(37)

    case object RotationRoll315Pitch315Yaw315 extends MAVSensorOrientation(38)

    val set: Set[MAVSensorOrientation] =
      Set(
        RotationNone,
        RotationYaw45,
        RotationYaw90,
        RotationYaw135,
        RotationYaw180,
        RotationYaw225,
        RotationYaw270,
        RotationYaw315,
        RotationRoll180,
        RotationRoll180Yaw45,
        RotationRoll180Yaw90,
        RotationRoll180Yaw135,
        RotationPitch180,
        RotationRoll180Yaw225,
        RotationRoll180Yaw270,
        RotationRoll180Yaw315,
        RotationRoll90,
        RotationRoll90Yaw45,
        RotationRoll90Yaw90,
        RotationRoll90Yaw135,
        RotationRoll270,
        RotationRoll270Yaw45,
        RotationRoll270Yaw90,
        RotationRoll270Yaw135,
        RotationPitch90,
        RotationPitch270,
        RotationPitch180Yaw90,
        RotationPitch180Yaw270,
        RotationRoll90Pitch90,
        RotationRoll180Pitch90,
        RotationRoll270Pitch90,
        RotationRoll90Pitch180,
        RotationRoll270Pitch180,
        RotationRoll90Pitch270,
        RotationRoll180Pitch270,
        RotationRoll270Pitch270,
        RotationRoll90Pitch180Yaw90,
        RotationRoll90Yaw270,
        RotationRoll315Pitch315Yaw315
      )

    val map: Map[Int, MAVSensorOrientation] = set.toSeq.map(instance => instance.value -> instance).toMap

    def apply(value: Int): MAVSensorOrientation = map.getOrElse(value, new MAVSensorOrientation(value))

    def bitmask(value: Int): Set[MAVSensorOrientation] = {
      val known = set.filter(instance => (instance.value & value) == instance.value)
      val sum = known.map(_.value).sum
      if (value == sum) known else known + new MAVSensorOrientation(value - sum)
    }
  }

  /**
    * MAV_PROTOCOL_CAPABILITY
    *
    * Bitmask of (optional) autopilot capabilities (64 bit). If a bit is set, the autopilot supports this capability.
    *
    * {{{
    * entry 1 MISSION_FLOAT Autopilot supports MISSION float message type.
    * entry 2 PARAM_FLOAT Autopilot supports the new param float message type.
    * entry 4 MISSION_INT Autopilot supports MISSION_INT scaled integer message type.
    * entry 8 COMMAND_INT Autopilot supports COMMAND_INT scaled integer message type.
    * entry 16 PARAM_UNION Autopilot supports the new param union message type.
    * entry 32 FTP Autopilot supports the new FILE_TRANSFER_PROTOCOL message type.
    * entry 64 SET_ATTITUDE_TARGET Autopilot supports commanding attitude offboard.
    * entry 128 SET_POSITION_TARGET_LOCAL_NED Autopilot supports commanding position and velocity targets in local NED frame.
    * entry 256 SET_POSITION_TARGET_GLOBAL_INT Autopilot supports commanding position and velocity targets in global scaled integers.
    * entry 512 TERRAIN Autopilot supports terrain protocol / data handling.
    * entry 1024 SET_ACTUATOR_TARGET Autopilot supports direct actuator control.
    * entry 2048 FLIGHT_TERMINATION Autopilot supports the flight termination command.
    * entry 4096 COMPASS_CALIBRATION Autopilot supports onboard compass calibration.
    * entry 8192 MAVLINK2 Autopilot supports mavlink version 2.
    * entry 16384 MISSION_FENCE Autopilot supports mission fence protocol.
    * entry 32768 MISSION_RALLY Autopilot supports mission rally point protocol.
    * entry 65536 FLIGHT_INFORMATION Autopilot supports the flight information protocol.
    * }}}
    */
  class MAVProtocolCapability(val value: Int)

  object MAVProtocolCapability {

    case object MissionFloat extends MAVProtocolCapability(1)

    case object ParamFloat extends MAVProtocolCapability(2)

    case object MissionInt extends MAVProtocolCapability(4)

    case object CommandInt extends MAVProtocolCapability(8)

    case object ParamUnion extends MAVProtocolCapability(16)

    case object FTP extends MAVProtocolCapability(32)

    case object SetAttitudeTarget extends MAVProtocolCapability(64)

    case object SetPositionTargetLocalNED extends MAVProtocolCapability(128)

    case object SetPositionTargetGlobalInt extends MAVProtocolCapability(256)

    case object Terrain extends MAVProtocolCapability(512)

    case object SetActuatorTarget extends MAVProtocolCapability(1024)

    case object FlightTermination extends MAVProtocolCapability(2048)

    case object CompassCalibration extends MAVProtocolCapability(4096)

    case object MAVLink2 extends MAVProtocolCapability(8192)

    case object MissionFence extends MAVProtocolCapability(16384)

    case object MissionRally extends MAVProtocolCapability(32768)

    case object FlightInformation extends MAVProtocolCapability(65536)

    val set: Set[MAVProtocolCapability] =
      Set(
        MissionFloat,
        ParamFloat,
        MissionInt,
        CommandInt,
        ParamUnion,
        FTP,
        SetAttitudeTarget,
        SetPositionTargetLocalNED,
        SetPositionTargetGlobalInt,
        Terrain,
        SetActuatorTarget,
        FlightTermination,
        CompassCalibration,
        MAVLink2,
        MissionFence,
        MissionRally,
        FlightInformation
      )

    val map: Map[Int, MAVProtocolCapability] = set.toSeq.map(instance => instance.value -> instance).toMap

    def apply(value: Int): MAVProtocolCapability = map.getOrElse(value, new MAVProtocolCapability(value))

    def bitmask(value: Int): Set[MAVProtocolCapability] = {
      val known = set.filter(instance => (instance.value & value) == instance.value)
      val sum = known.map(_.value).sum
      if (value == sum) known else known + new MAVProtocolCapability(value - sum)
    }
  }

  /**
    * MAV_MISSION_TYPE
    *
    * Type of mission items being requested/sent in mission protocol.
    *
    * {{{
    * entry 0 MISSION Items are mission commands for main mission.
    * entry 1 FENCE Specifies GeoFence area(s). Items are MAV_CMD_FENCE_ GeoFence items.
    * entry 2 RALLY Specifies the rally points for the vehicle. Rally points are alternative RTL points. Items are MAV_CMD_RALLY_POINT rally point items.
    * entry 255 ALL Only used in MISSION_CLEAR_ALL to clear all mission types.
    * }}}
    */
  class MAVMissionType(val value: Int)

  object MAVMissionType {

    case object Mission extends MAVMissionType(0)

    case object Fence extends MAVMissionType(1)

    case object Rally extends MAVMissionType(2)

    case object All extends MAVMissionType(255)

    val set: Set[MAVMissionType] =
      Set(
        Mission,
        Fence,
        Rally,
        All
      )

    val map: Map[Int, MAVMissionType] = set.toSeq.map(instance => instance.value -> instance).toMap

    def apply(value: Int): MAVMissionType = map.getOrElse(value, new MAVMissionType(value))

    def bitmask(value: Int): Set[MAVMissionType] = {
      val known = set.filter(instance => (instance.value & value) == instance.value)
      val sum = known.map(_.value).sum
      if (value == sum) known else known + new MAVMissionType(value - sum)
    }
  }

  /**
    * MAV_ESTIMATOR_TYPE
    *
    * Enumeration of estimator types
    *
    * {{{
    * entry 1 NAIVE This is a naive estimator without any real covariance feedback.
    * entry 2 VISION Computer vision based estimate. Might be up to scale.
    * entry 3 VIO Visual-inertial estimate.
    * entry 4 GPS Plain GPS estimate.
    * entry 5 GPS_INS Estimator integrating GPS and inertial sensing.
    * }}}
    */
  class MAVEstimatorType(val value: Int)

  object MAVEstimatorType {

    case object Naive extends MAVEstimatorType(1)

    case object Vision extends MAVEstimatorType(2)

    case object VIO extends MAVEstimatorType(3)

    case object GPS extends MAVEstimatorType(4)

    case object GPSIns extends MAVEstimatorType(5)

    val set: Set[MAVEstimatorType] =
      Set(
        Naive,
        Vision,
        VIO,
        GPS,
        GPSIns
      )

    val map: Map[Int, MAVEstimatorType] = set.toSeq.map(instance => instance.value -> instance).toMap

    def apply(value: Int): MAVEstimatorType = map.getOrElse(value, new MAVEstimatorType(value))

    def bitmask(value: Int): Set[MAVEstimatorType] = {
      val known = set.filter(instance => (instance.value & value) == instance.value)
      val sum = known.map(_.value).sum
      if (value == sum) known else known + new MAVEstimatorType(value - sum)
    }
  }

  /**
    * MAV_BATTERY_TYPE
    *
    * Enumeration of battery types
    *
    * {{{
    * entry 0 UNKNOWN Not specified.
    * entry 1 LIPO Lithium polymer battery
    * entry 2 LIFE Lithium-iron-phosphate battery
    * entry 3 LION Lithium-ION battery
    * entry 4 NIMH Nickel metal hydride battery
    * }}}
    */
  class MAVBatteryType(val value: Int)

  object MAVBatteryType {

    case object Unknown extends MAVBatteryType(0)

    case object LiPo extends MAVBatteryType(1)

    case object LiFe extends MAVBatteryType(2)

    case object Lion extends MAVBatteryType(3)

    case object NiMH extends MAVBatteryType(4)

    val set: Set[MAVBatteryType] =
      Set(
        Unknown,
        LiPo,
        LiFe,
        Lion,
        NiMH
      )

    val map: Map[Int, MAVBatteryType] = set.toSeq.map(instance => instance.value -> instance).toMap

    def apply(value: Int): MAVBatteryType = map.getOrElse(value, new MAVBatteryType(value))

    def bitmask(value: Int): Set[MAVBatteryType] = {
      val known = set.filter(instance => (instance.value & value) == instance.value)
      val sum = known.map(_.value).sum
      if (value == sum) known else known + new MAVBatteryType(value - sum)
    }
  }

  /**
    * MAV_BATTERY_FUNCTION
    *
    * Enumeration of battery functions
    *
    * {{{
    * entry 0 FUNCTION_UNKNOWN Battery function is unknown
    * entry 1 FUNCTION_ALL Battery supports all flight systems
    * entry 2 FUNCTION_PROPULSION Battery for the propulsion system
    * entry 3 FUNCTION_AVIONICS Avionics battery
    * entry 4 TYPE_PAYLOAD Payload battery
    * }}}
    */
  class MAVBatteryFunction(val value: Int)

  object MAVBatteryFunction {

    case object FunctionUnknown extends MAVBatteryFunction(0)

    case object FunctionAll extends MAVBatteryFunction(1)

    case object FunctionPropulsion extends MAVBatteryFunction(2)

    case object FunctionAvionics extends MAVBatteryFunction(3)

    case object TypePayload extends MAVBatteryFunction(4)

    val set: Set[MAVBatteryFunction] =
      Set(
        FunctionUnknown,
        FunctionAll,
        FunctionPropulsion,
        FunctionAvionics,
        TypePayload
      )

    val map: Map[Int, MAVBatteryFunction] = set.toSeq.map(instance => instance.value -> instance).toMap

    def apply(value: Int): MAVBatteryFunction = map.getOrElse(value, new MAVBatteryFunction(value))

    def bitmask(value: Int): Set[MAVBatteryFunction] = {
      val known = set.filter(instance => (instance.value & value) == instance.value)
      val sum = known.map(_.value).sum
      if (value == sum) known else known + new MAVBatteryFunction(value - sum)
    }
  }

  /**
    * MAV_VTOL_STATE
    *
    * Enumeration of VTOL states
    *
    * {{{
    * entry 0 UNDEFINED MAV is not configured as VTOL
    * entry 1 TRANSITION_TO_FW VTOL is in transition from multicopter to fixed-wing
    * entry 2 TRANSITION_TO_MC VTOL is in transition from fixed-wing to multicopter
    * entry 3 MC VTOL is in multicopter state
    * entry 4 FW VTOL is in fixed-wing state
    * }}}
    */
  class MAVVTOLState(val value: Int)

  object MAVVTOLState {

    case object Undefined extends MAVVTOLState(0)

    case object TransitionToFW extends MAVVTOLState(1)

    case object TransitionToMC extends MAVVTOLState(2)

    case object MC extends MAVVTOLState(3)

    case object FW extends MAVVTOLState(4)

    val set: Set[MAVVTOLState] =
      Set(
        Undefined,
        TransitionToFW,
        TransitionToMC,
        MC,
        FW
      )

    val map: Map[Int, MAVVTOLState] = set.toSeq.map(instance => instance.value -> instance).toMap

    def apply(value: Int): MAVVTOLState = map.getOrElse(value, new MAVVTOLState(value))

    def bitmask(value: Int): Set[MAVVTOLState] = {
      val known = set.filter(instance => (instance.value & value) == instance.value)
      val sum = known.map(_.value).sum
      if (value == sum) known else known + new MAVVTOLState(value - sum)
    }
  }

  /**
    * MAV_LANDED_STATE
    *
    * Enumeration of landed detector states
    *
    * {{{
    * entry 0 UNDEFINED MAV landed state is unknown
    * entry 1 ON_GROUND MAV is landed (on ground)
    * entry 2 IN_AIR MAV is in air
    * entry 3 TAKEOFF MAV currently taking off
    * entry 4 LANDING MAV currently landing
    * }}}
    */
  class MAVLandedState(val value: Int)

  object MAVLandedState {

    case object Undefined extends MAVLandedState(0)

    case object OnGround extends MAVLandedState(1)

    case object InAir extends MAVLandedState(2)

    case object Takeoff extends MAVLandedState(3)

    case object Landing extends MAVLandedState(4)

    val set: Set[MAVLandedState] =
      Set(
        Undefined,
        OnGround,
        InAir,
        Takeoff,
        Landing
      )

    val map: Map[Int, MAVLandedState] = set.toSeq.map(instance => instance.value -> instance).toMap

    def apply(value: Int): MAVLandedState = map.getOrElse(value, new MAVLandedState(value))

    def bitmask(value: Int): Set[MAVLandedState] = {
      val known = set.filter(instance => (instance.value & value) == instance.value)
      val sum = known.map(_.value).sum
      if (value == sum) known else known + new MAVLandedState(value - sum)
    }
  }

  /**
    * ADSB_ALTITUDE_TYPE
    *
    * Enumeration of the ADSB altimeter types
    *
    * {{{
    * entry 0 PRESSURE_QNH Altitude reported from a Baro source using QNH reference
    * entry 1 GEOMETRIC Altitude reported from a GNSS source
    * }}}
    */
  class ADSBAltitudeType(val value: Int)

  object ADSBAltitudeType {

    case object PressureQNH extends ADSBAltitudeType(0)

    case object Geometric extends ADSBAltitudeType(1)

    val set: Set[ADSBAltitudeType] =
      Set(
        PressureQNH,
        Geometric
      )

    val map: Map[Int, ADSBAltitudeType] = set.toSeq.map(instance => instance.value -> instance).toMap

    def apply(value: Int): ADSBAltitudeType = map.getOrElse(value, new ADSBAltitudeType(value))

    def bitmask(value: Int): Set[ADSBAltitudeType] = {
      val known = set.filter(instance => (instance.value & value) == instance.value)
      val sum = known.map(_.value).sum
      if (value == sum) known else known + new ADSBAltitudeType(value - sum)
    }
  }

  /**
    * ADSB_EMITTER_TYPE
    *
    * ADSB classification for the type of vehicle emitting the transponder signal
    *
    * {{{
    * entry 0 NO_INFO
    * entry 1 LIGHT
    * entry 2 SMALL
    * entry 3 LARGE
    * entry 4 HIGH_VORTEX_LARGE
    * entry 5 HEAVY
    * entry 6 HIGHLY_MANUV
    * entry 7 ROTOCRAFT
    * entry 8 UNASSIGNED
    * entry 9 GLIDER
    * entry 10 LIGHTER_AIR
    * entry 11 PARACHUTE
    * entry 12 ULTRA_LIGHT
    * entry 13 UNASSIGNED2
    * entry 14 UAV
    * entry 15 SPACE
    * entry 16 UNASSGINED3
    * entry 17 EMERGENCY_SURFACE
    * entry 18 SERVICE_SURFACE
    * entry 19 POINT_OBSTACLE
    * }}}
    */
  class ADSBEmitterType(val value: Int)

  object ADSBEmitterType {

    case object NoInfo extends ADSBEmitterType(0)

    case object Light extends ADSBEmitterType(1)

    case object Small extends ADSBEmitterType(2)

    case object Large extends ADSBEmitterType(3)

    case object HighVortexLarge extends ADSBEmitterType(4)

    case object Heavy extends ADSBEmitterType(5)

    case object HighlyManuv extends ADSBEmitterType(6)

    case object Rotocraft extends ADSBEmitterType(7)

    case object Unassigned extends ADSBEmitterType(8)

    case object Glider extends ADSBEmitterType(9)

    case object LighterAir extends ADSBEmitterType(10)

    case object Parachute extends ADSBEmitterType(11)

    case object UltraLight extends ADSBEmitterType(12)

    case object Unassigned2 extends ADSBEmitterType(13)

    case object UAV extends ADSBEmitterType(14)

    case object Space extends ADSBEmitterType(15)

    case object Unassgined3 extends ADSBEmitterType(16)

    case object EmergencySurface extends ADSBEmitterType(17)

    case object ServiceSurface extends ADSBEmitterType(18)

    case object PointObstacle extends ADSBEmitterType(19)

    val set: Set[ADSBEmitterType] =
      Set(
        NoInfo,
        Light,
        Small,
        Large,
        HighVortexLarge,
        Heavy,
        HighlyManuv,
        Rotocraft,
        Unassigned,
        Glider,
        LighterAir,
        Parachute,
        UltraLight,
        Unassigned2,
        UAV,
        Space,
        Unassgined3,
        EmergencySurface,
        ServiceSurface,
        PointObstacle
      )

    val map: Map[Int, ADSBEmitterType] = set.toSeq.map(instance => instance.value -> instance).toMap

    def apply(value: Int): ADSBEmitterType = map.getOrElse(value, new ADSBEmitterType(value))

    def bitmask(value: Int): Set[ADSBEmitterType] = {
      val known = set.filter(instance => (instance.value & value) == instance.value)
      val sum = known.map(_.value).sum
      if (value == sum) known else known + new ADSBEmitterType(value - sum)
    }
  }

  /**
    * ADSB_FLAGS
    *
    * These flags indicate status such as data validity of each data source. Set = data valid
    *
    * {{{
    * entry 1 VALID_COORDS
    * entry 2 VALID_ALTITUDE
    * entry 4 VALID_HEADING
    * entry 8 VALID_VELOCITY
    * entry 16 VALID_CALLSIGN
    * entry 32 VALID_SQUAWK
    * entry 64 SIMULATED
    * }}}
    */
  class ADSBFlags(val value: Int)

  object ADSBFlags {

    case object ValidCoords extends ADSBFlags(1)

    case object ValidAltitude extends ADSBFlags(2)

    case object ValidHeading extends ADSBFlags(4)

    case object ValidVelocity extends ADSBFlags(8)

    case object ValidCallsign extends ADSBFlags(16)

    case object ValidSquawk extends ADSBFlags(32)

    case object Simulated extends ADSBFlags(64)

    val set: Set[ADSBFlags] =
      Set(
        ValidCoords,
        ValidAltitude,
        ValidHeading,
        ValidVelocity,
        ValidCallsign,
        ValidSquawk,
        Simulated
      )

    val map: Map[Int, ADSBFlags] = set.toSeq.map(instance => instance.value -> instance).toMap

    def apply(value: Int): ADSBFlags = map.getOrElse(value, new ADSBFlags(value))

    def bitmask(value: Int): Set[ADSBFlags] = {
      val known = set.filter(instance => (instance.value & value) == instance.value)
      val sum = known.map(_.value).sum
      if (value == sum) known else known + new ADSBFlags(value - sum)
    }
  }

  /**
    * MAV_DO_REPOSITION_FLAGS
    *
    * Bitmask of options for the MAV_CMD_DO_REPOSITION
    *
    * {{{
    * entry 1 CHANGE_MODE The aircraft should immediately transition into guided. This should not be set for follow me applications
    * }}}
    */
  class MAVDoRepositionFlags(val value: Int)

  object MAVDoRepositionFlags {

    case object ChangeMode extends MAVDoRepositionFlags(1)

    val set: Set[MAVDoRepositionFlags] =
      Set(
        ChangeMode
      )

    val map: Map[Int, MAVDoRepositionFlags] = set.toSeq.map(instance => instance.value -> instance).toMap

    def apply(value: Int): MAVDoRepositionFlags = map.getOrElse(value, new MAVDoRepositionFlags(value))

    def bitmask(value: Int): Set[MAVDoRepositionFlags] = {
      val known = set.filter(instance => (instance.value & value) == instance.value)
      val sum = known.map(_.value).sum
      if (value == sum) known else known + new MAVDoRepositionFlags(value - sum)
    }
  }

  /**
    * ESTIMATOR_STATUS_FLAGS
    *
    * Flags in EKF_STATUS message
    *
    * {{{
    * entry 1 ATTITUDE True if the attitude estimate is good
    * entry 2 VELOCITY_HORIZ True if the horizontal velocity estimate is good
    * entry 4 VELOCITY_VERT True if the  vertical velocity estimate is good
    * entry 8 POS_HORIZ_REL True if the horizontal position (relative) estimate is good
    * entry 16 POS_HORIZ_ABS True if the horizontal position (absolute) estimate is good
    * entry 32 POS_VERT_ABS True if the vertical position (absolute) estimate is good
    * entry 64 POS_VERT_AGL True if the vertical position (above ground) estimate is good
    * entry 128 CONST_POS_MODE True if the EKF is in a constant position mode and is not using external measurements (eg GPS or optical flow)
    * entry 256 PRED_POS_HORIZ_REL True if the EKF has sufficient data to enter a mode that will provide a (relative) position estimate
    * entry 512 PRED_POS_HORIZ_ABS True if the EKF has sufficient data to enter a mode that will provide a (absolute) position estimate
    * entry 1024 GPS_GLITCH True if the EKF has detected a GPS glitch
    * }}}
    */
  class EstimatorStatusFlags(val value: Int)

  object EstimatorStatusFlags {

    case object Attitude extends EstimatorStatusFlags(1)

    case object VelocityHoriz extends EstimatorStatusFlags(2)

    case object VelocityVert extends EstimatorStatusFlags(4)

    case object PosHorizRel extends EstimatorStatusFlags(8)

    case object PosHorizAbs extends EstimatorStatusFlags(16)

    case object PosVertAbs extends EstimatorStatusFlags(32)

    case object PosVertAgl extends EstimatorStatusFlags(64)

    case object ConstPosMode extends EstimatorStatusFlags(128)

    case object PredPosHorizRel extends EstimatorStatusFlags(256)

    case object PredPosHorizAbs extends EstimatorStatusFlags(512)

    case object GPSGlitch extends EstimatorStatusFlags(1024)

    val set: Set[EstimatorStatusFlags] =
      Set(
        Attitude,
        VelocityHoriz,
        VelocityVert,
        PosHorizRel,
        PosHorizAbs,
        PosVertAbs,
        PosVertAgl,
        ConstPosMode,
        PredPosHorizRel,
        PredPosHorizAbs,
        GPSGlitch
      )

    val map: Map[Int, EstimatorStatusFlags] = set.toSeq.map(instance => instance.value -> instance).toMap

    def apply(value: Int): EstimatorStatusFlags = map.getOrElse(value, new EstimatorStatusFlags(value))

    def bitmask(value: Int): Set[EstimatorStatusFlags] = {
      val known = set.filter(instance => (instance.value & value) == instance.value)
      val sum = known.map(_.value).sum
      if (value == sum) known else known + new EstimatorStatusFlags(value - sum)
    }
  }

  /**
    * MOTOR_TEST_THROTTLE_TYPE
    *
    * {{{
    * entry 0 PERCENT throttle as a percentage from 0 ~ 100
    * entry 1 PWM throttle as an absolute PWM value (normally in range of 1000~2000)
    * entry 2 PILOT throttle pass-through from pilot's transmitter
    * }}}
    */
  class MotorTestThrottleType(val value: Int)

  object MotorTestThrottleType {

    case object Percent extends MotorTestThrottleType(0)

    case object PWM extends MotorTestThrottleType(1)

    case object Pilot extends MotorTestThrottleType(2)

    val set: Set[MotorTestThrottleType] =
      Set(
        Percent,
        PWM,
        Pilot
      )

    val map: Map[Int, MotorTestThrottleType] = set.toSeq.map(instance => instance.value -> instance).toMap

    def apply(value: Int): MotorTestThrottleType = map.getOrElse(value, new MotorTestThrottleType(value))

    def bitmask(value: Int): Set[MotorTestThrottleType] = {
      val known = set.filter(instance => (instance.value & value) == instance.value)
      val sum = known.map(_.value).sum
      if (value == sum) known else known + new MotorTestThrottleType(value - sum)
    }
  }

  /**
    * GPS_INPUT_IGNORE_FLAGS
    *
    * {{{
    * entry 1 ALT ignore altitude field
    * entry 2 HDOP ignore hdop field
    * entry 4 VDOP ignore vdop field
    * entry 8 VEL_HORIZ ignore horizontal velocity field (vn and ve)
    * entry 16 VEL_VERT ignore vertical velocity field (vd)
    * entry 32 SPEED_ACCURACY ignore speed accuracy field
    * entry 64 HORIZONTAL_ACCURACY ignore horizontal accuracy field
    * entry 128 VERTICAL_ACCURACY ignore vertical accuracy field
    * }}}
    */
  class GPSInputIgnoreFlags(val value: Int)

  object GPSInputIgnoreFlags {

    case object Alt extends GPSInputIgnoreFlags(1)

    case object HDOP extends GPSInputIgnoreFlags(2)

    case object VDOP extends GPSInputIgnoreFlags(4)

    case object VelHoriz extends GPSInputIgnoreFlags(8)

    case object VelVert extends GPSInputIgnoreFlags(16)

    case object SpeedAccuracy extends GPSInputIgnoreFlags(32)

    case object HorizontalAccuracy extends GPSInputIgnoreFlags(64)

    case object VerticalAccuracy extends GPSInputIgnoreFlags(128)

    val set: Set[GPSInputIgnoreFlags] =
      Set(
        Alt,
        HDOP,
        VDOP,
        VelHoriz,
        VelVert,
        SpeedAccuracy,
        HorizontalAccuracy,
        VerticalAccuracy
      )

    val map: Map[Int, GPSInputIgnoreFlags] = set.toSeq.map(instance => instance.value -> instance).toMap

    def apply(value: Int): GPSInputIgnoreFlags = map.getOrElse(value, new GPSInputIgnoreFlags(value))

    def bitmask(value: Int): Set[GPSInputIgnoreFlags] = {
      val known = set.filter(instance => (instance.value & value) == instance.value)
      val sum = known.map(_.value).sum
      if (value == sum) known else known + new GPSInputIgnoreFlags(value - sum)
    }
  }

  /**
    * MAV_COLLISION_ACTION
    *
    * Possible actions an aircraft can take to avoid a collision.
    *
    * {{{
    * entry 0 NONE Ignore any potential collisions
    * entry 1 REPORT Report potential collision
    * entry 2 ASCEND_OR_DESCEND Ascend or Descend to avoid threat
    * entry 3 MOVE_HORIZONTALLY Move horizontally to avoid threat
    * entry 4 MOVE_PERPENDICULAR Aircraft to move perpendicular to the collision's velocity vector
    * entry 5 RTL Aircraft to fly directly back to its launch point
    * entry 6 HOVER Aircraft to stop in place
    * }}}
    */
  class MAVCollisionAction(val value: Int)

  object MAVCollisionAction {

    case object None extends MAVCollisionAction(0)

    case object Report extends MAVCollisionAction(1)

    case object AscendOrDescend extends MAVCollisionAction(2)

    case object MoveHorizontally extends MAVCollisionAction(3)

    case object MovePerpendicular extends MAVCollisionAction(4)

    case object RTL extends MAVCollisionAction(5)

    case object Hover extends MAVCollisionAction(6)

    val set: Set[MAVCollisionAction] =
      Set(
        None,
        Report,
        AscendOrDescend,
        MoveHorizontally,
        MovePerpendicular,
        RTL,
        Hover
      )

    val map: Map[Int, MAVCollisionAction] = set.toSeq.map(instance => instance.value -> instance).toMap

    def apply(value: Int): MAVCollisionAction = map.getOrElse(value, new MAVCollisionAction(value))

    def bitmask(value: Int): Set[MAVCollisionAction] = {
      val known = set.filter(instance => (instance.value & value) == instance.value)
      val sum = known.map(_.value).sum
      if (value == sum) known else known + new MAVCollisionAction(value - sum)
    }
  }

  /**
    * MAV_COLLISION_THREAT_LEVEL
    *
    * Aircraft-rated danger from this threat.
    *
    * {{{
    * entry 0 NONE Not a threat
    * entry 1 LOW Craft is mildly concerned about this threat
    * entry 2 HIGH Craft is panicing, and may take actions to avoid threat
    * }}}
    */
  class MAVCollisionThreatLevel(val value: Int)

  object MAVCollisionThreatLevel {

    case object None extends MAVCollisionThreatLevel(0)

    case object Low extends MAVCollisionThreatLevel(1)

    case object High extends MAVCollisionThreatLevel(2)

    val set: Set[MAVCollisionThreatLevel] =
      Set(
        None,
        Low,
        High
      )

    val map: Map[Int, MAVCollisionThreatLevel] = set.toSeq.map(instance => instance.value -> instance).toMap

    def apply(value: Int): MAVCollisionThreatLevel = map.getOrElse(value, new MAVCollisionThreatLevel(value))

    def bitmask(value: Int): Set[MAVCollisionThreatLevel] = {
      val known = set.filter(instance => (instance.value & value) == instance.value)
      val sum = known.map(_.value).sum
      if (value == sum) known else known + new MAVCollisionThreatLevel(value - sum)
    }
  }

  /**
    * MAV_COLLISION_SRC
    *
    * Source of information about this collision.
    *
    * {{{
    * entry 0 ADSB ID field references ADSB_VEHICLE packets
    * entry 1 MAVLINK_GPS_GLOBAL_INT ID field references MAVLink SRC ID
    * }}}
    */
  class MAVCollisionSrc(val value: Int)

  object MAVCollisionSrc {

    case object ADSB extends MAVCollisionSrc(0)

    case object MAVLinkGPSGlobalInt extends MAVCollisionSrc(1)

    val set: Set[MAVCollisionSrc] =
      Set(
        ADSB,
        MAVLinkGPSGlobalInt
      )

    val map: Map[Int, MAVCollisionSrc] = set.toSeq.map(instance => instance.value -> instance).toMap

    def apply(value: Int): MAVCollisionSrc = map.getOrElse(value, new MAVCollisionSrc(value))

    def bitmask(value: Int): Set[MAVCollisionSrc] = {
      val known = set.filter(instance => (instance.value & value) == instance.value)
      val sum = known.map(_.value).sum
      if (value == sum) known else known + new MAVCollisionSrc(value - sum)
    }
  }

  /**
    * GPS_FIX_TYPE
    *
    * Type of GPS fix
    *
    * {{{
    * entry 0 NO_GPS No GPS connected
    * entry 1 NO_FIX No position information, GPS is connected
    * entry 2 2D_FIX 2D position
    * entry 3 3D_FIX 3D position
    * entry 4 DGPS DGPS/SBAS aided 3D position
    * entry 5 RTK_FLOAT RTK float, 3D position
    * entry 6 RTK_FIXED RTK Fixed, 3D position
    * entry 7 STATIC Static fixed, typically used for base stations
    * entry 8 PPP PPP, 3D position.
    * }}}
    */
  class GPSFixType(val value: Int)

  object GPSFixType {

    case object NoGPS extends GPSFixType(0)

    case object NoFix extends GPSFixType(1)

    case object _2dFix extends GPSFixType(2)

    case object _3dFix extends GPSFixType(3)

    case object DGPS extends GPSFixType(4)

    case object RTKFloat extends GPSFixType(5)

    case object RTKFixed extends GPSFixType(6)

    case object Static extends GPSFixType(7)

    case object PPP extends GPSFixType(8)

    val set: Set[GPSFixType] =
      Set(
        NoGPS,
        NoFix,
        _2dFix,
        _3dFix,
        DGPS,
        RTKFloat,
        RTKFixed,
        Static,
        PPP
      )

    val map: Map[Int, GPSFixType] = set.toSeq.map(instance => instance.value -> instance).toMap

    def apply(value: Int): GPSFixType = map.getOrElse(value, new GPSFixType(value))

    def bitmask(value: Int): Set[GPSFixType] = {
      val known = set.filter(instance => (instance.value & value) == instance.value)
      val sum = known.map(_.value).sum
      if (value == sum) known else known + new GPSFixType(value - sum)
    }
  }

  /**
    * LANDING_TARGET_TYPE
    *
    * Type of landing target
    *
    * {{{
    * entry 0 LIGHT_BEACON Landing target signaled by light beacon (ex: IR-LOCK)
    * entry 1 RADIO_BEACON Landing target signaled by radio beacon (ex: ILS, NDB)
    * entry 2 VISION_FIDUCIAL Landing target represented by a fiducial marker (ex: ARTag)
    * entry 3 VISION_OTHER Landing target represented by a pre-defined visual shape/feature (ex: X-marker, H-marker, square)
    * }}}
    */
  class LandingTargetType(val value: Int)

  object LandingTargetType {

    case object LightBeacon extends LandingTargetType(0)

    case object RadioBeacon extends LandingTargetType(1)

    case object VisionFiducial extends LandingTargetType(2)

    case object VisionOther extends LandingTargetType(3)

    val set: Set[LandingTargetType] =
      Set(
        LightBeacon,
        RadioBeacon,
        VisionFiducial,
        VisionOther
      )

    val map: Map[Int, LandingTargetType] = set.toSeq.map(instance => instance.value -> instance).toMap

    def apply(value: Int): LandingTargetType = map.getOrElse(value, new LandingTargetType(value))

    def bitmask(value: Int): Set[LandingTargetType] = {
      val known = set.filter(instance => (instance.value & value) == instance.value)
      val sum = known.map(_.value).sum
      if (value == sum) known else known + new LandingTargetType(value - sum)
    }
  }

  /**
    * VTOL_TRANSITION_HEADING
    *
    * Direction of VTOL transition
    *
    * {{{
    * entry 0 VEHICLE_DEFAULT Respect the heading configuration of the vehicle.
    * entry 1 NEXT_WAYPOINT Use the heading pointing towards the next waypoint.
    * entry 2 TAKEOFF Use the heading on takeoff (while sitting on the ground).
    * entry 3 SPECIFIED Use the specified heading in parameter 4.
    * entry 4 ANY Use the current heading when reaching takeoff altitude (potentially facing the wind when weather-vaning is active).
    * }}}
    */
  class VTOLTransitionHeading(val value: Int)

  object VTOLTransitionHeading {

    case object VehicleDefault extends VTOLTransitionHeading(0)

    case object NextWaypoint extends VTOLTransitionHeading(1)

    case object Takeoff extends VTOLTransitionHeading(2)

    case object Specified extends VTOLTransitionHeading(3)

    case object Any extends VTOLTransitionHeading(4)

    val set: Set[VTOLTransitionHeading] =
      Set(
        VehicleDefault,
        NextWaypoint,
        Takeoff,
        Specified,
        Any
      )

    val map: Map[Int, VTOLTransitionHeading] = set.toSeq.map(instance => instance.value -> instance).toMap

    def apply(value: Int): VTOLTransitionHeading = map.getOrElse(value, new VTOLTransitionHeading(value))

    def bitmask(value: Int): Set[VTOLTransitionHeading] = {
      val known = set.filter(instance => (instance.value & value) == instance.value)
      val sum = known.map(_.value).sum
      if (value == sum) known else known + new VTOLTransitionHeading(value - sum)
    }
  }

  /**
    * CAMERA_CAP_FLAGS
    *
    * Camera capability flags (Bitmap).
    *
    * {{{
    * entry 1 CAPTURE_VIDEO Camera is able to record video.
    * entry 2 CAPTURE_IMAGE Camera is able to capture images.
    * entry 4 HAS_MODES Camera has separate Video and Image/Photo modes (MAV_CMD_SET_CAMERA_MODE)
    * entry 8 CAN_CAPTURE_IMAGE_IN_VIDEO_MODE Camera can capture images while in video mode
    * entry 16 CAN_CAPTURE_VIDEO_IN_IMAGE_MODE Camera can capture videos while in Photo/Image mode
    * entry 32 HAS_IMAGE_SURVEY_MODE Camera has image survey mode (MAV_CMD_SET_CAMERA_MODE)
    * }}}
    */
  class CameraCapFlags(val value: Int)

  object CameraCapFlags {

    case object CaptureVideo extends CameraCapFlags(1)

    case object CaptureImage extends CameraCapFlags(2)

    case object HasModes extends CameraCapFlags(4)

    case object CanCaptureImageInVideoMode extends CameraCapFlags(8)

    case object CanCaptureVideoInImageMode extends CameraCapFlags(16)

    case object HasImageSurveyMode extends CameraCapFlags(32)

    val set: Set[CameraCapFlags] =
      Set(
        CaptureVideo,
        CaptureImage,
        HasModes,
        CanCaptureImageInVideoMode,
        CanCaptureVideoInImageMode,
        HasImageSurveyMode
      )

    val map: Map[Int, CameraCapFlags] = set.toSeq.map(instance => instance.value -> instance).toMap

    def apply(value: Int): CameraCapFlags = map.getOrElse(value, new CameraCapFlags(value))

    def bitmask(value: Int): Set[CameraCapFlags] = {
      val known = set.filter(instance => (instance.value & value) == instance.value)
      val sum = known.map(_.value).sum
      if (value == sum) known else known + new CameraCapFlags(value - sum)
    }
  }

  /**
    * PARAM_ACK
    *
    * Result from a PARAM_EXT_SET message.
    *
    * {{{
    * entry 0 ACCEPTED Parameter value ACCEPTED and SET
    * entry 1 VALUE_UNSUPPORTED Parameter value UNKNOWN/UNSUPPORTED
    * entry 2 FAILED Parameter failed to set
    * entry 3 IN_PROGRESS Parameter value received but not yet validated or set. A subsequent PARAM_EXT_ACK will follow once operation is completed with the actual result. These are for parameters that may take longer to set. Instead of waiting for an ACK and potentially timing out, you will immediately receive this response to let you know it was received.
    * }}}
    */
  class ParamAck(val value: Int)

  object ParamAck {

    case object Accepted extends ParamAck(0)

    case object ValueUnsupported extends ParamAck(1)

    case object Failed extends ParamAck(2)

    case object InProgress extends ParamAck(3)

    val set: Set[ParamAck] =
      Set(
        Accepted,
        ValueUnsupported,
        Failed,
        InProgress
      )

    val map: Map[Int, ParamAck] = set.toSeq.map(instance => instance.value -> instance).toMap

    def apply(value: Int): ParamAck = map.getOrElse(value, new ParamAck(value))

    def bitmask(value: Int): Set[ParamAck] = {
      val known = set.filter(instance => (instance.value & value) == instance.value)
      val sum = known.map(_.value).sum
      if (value == sum) known else known + new ParamAck(value - sum)
    }
  }

  /**
    * CAMERA_MODE
    *
    * Camera Modes.
    *
    * {{{
    * entry 0 IMAGE Camera is in image/photo capture mode.
    * entry 1 VIDEO Camera is in video capture mode.
    * entry 2 IMAGE_SURVEY Camera is in image survey capture mode. It allows for camera controller to do specific settings for surveys.
    * }}}
    */
  class CameraMode(val value: Int)

  object CameraMode {

    case object Image extends CameraMode(0)

    case object Video extends CameraMode(1)

    case object ImageSurvey extends CameraMode(2)

    val set: Set[CameraMode] =
      Set(
        Image,
        Video,
        ImageSurvey
      )

    val map: Map[Int, CameraMode] = set.toSeq.map(instance => instance.value -> instance).toMap

    def apply(value: Int): CameraMode = map.getOrElse(value, new CameraMode(value))

    def bitmask(value: Int): Set[CameraMode] = {
      val known = set.filter(instance => (instance.value & value) == instance.value)
      val sum = known.map(_.value).sum
      if (value == sum) known else known + new CameraMode(value - sum)
    }
  }

  /**
    * MAV_ARM_AUTH_DENIED_REASON
    *
    * {{{
    * entry 0 GENERIC Not a specific reason
    * entry 1 NONE Authorizer will send the error as string to GCS
    * entry 2 INVALID_WAYPOINT At least one waypoint have a invalid value
    * entry 3 TIMEOUT Timeout in the authorizer process(in case it depends on network)
    * entry 4 AIRSPACE_IN_USE Airspace of the mission in use by another vehicle, second result parameter can have the waypoint id that caused it to be denied.
    * entry 5 BAD_WEATHER Weather is not good to fly
    * }}}
    */
  class MAVArmAuthDeniedReason(val value: Int)

  object MAVArmAuthDeniedReason {

    case object Generic extends MAVArmAuthDeniedReason(0)

    case object None extends MAVArmAuthDeniedReason(1)

    case object InvalidWaypoint extends MAVArmAuthDeniedReason(2)

    case object Timeout extends MAVArmAuthDeniedReason(3)

    case object AirspaceInUse extends MAVArmAuthDeniedReason(4)

    case object BadWeather extends MAVArmAuthDeniedReason(5)

    val set: Set[MAVArmAuthDeniedReason] =
      Set(
        Generic,
        None,
        InvalidWaypoint,
        Timeout,
        AirspaceInUse,
        BadWeather
      )

    val map: Map[Int, MAVArmAuthDeniedReason] = set.toSeq.map(instance => instance.value -> instance).toMap

    def apply(value: Int): MAVArmAuthDeniedReason = map.getOrElse(value, new MAVArmAuthDeniedReason(value))

    def bitmask(value: Int): Set[MAVArmAuthDeniedReason] = {
      val known = set.filter(instance => (instance.value & value) == instance.value)
      val sum = known.map(_.value).sum
      if (value == sum) known else known + new MAVArmAuthDeniedReason(value - sum)
    }
  }

}
