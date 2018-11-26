package frc.team321.robot

import com.team254.lib.physics.DCMotorTransmission
import com.team254.lib.physics.DifferentialDrive
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.units.degree
import org.ghrobotics.lib.mathematics.units.feet
import org.ghrobotics.lib.mathematics.units.inch
import org.ghrobotics.lib.mathematics.units.meter
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitLengthModel
import org.ghrobotics.lib.mathematics.units.nativeunits.STU
import kotlin.math.pow

@Suppress("MemberVisibilityCanBePrivate", "unused")
object Constants {

    const val TIMEOUT = 0

    const val JOYSTICK_DEADBAND = 0.1

    object Drivetrain {
        const val PID_SLOT_INDEX = 0
        const val PROFILE_SLOT_INDEX = 0

        const val LEFT_KF = 1.21
        const val RIGHT_KF = 1.17

        const val TURN_KP = 0.01
        const val TURN_KI = 0.0
        const val TURN_KD = 0.1

        const val VOLTAGE_COMPENSATION = 12.0 //Volt
        const val FILTER_WINDOW_SAMPLES = 32

        const val MAX_VELOCITY = 12.0 //Feet per second
        const val MAX_ACCELERATION = 30.0 // Feet per second^2

        const val kStatic = 0.1
        const val kV = 1 / MAX_VELOCITY
        const val kA = 1.0 / MAX_ACCELERATION

        const val TICKS_PER_REVOLUTION = 1024

        const val WHEEL_DIAMETER = 0.51 //Feet

        const val RAMP_RATE = 0.25

        const val WHEELBASE = 2.25

        //Poof Constants
        const val kRobotMass = 54.53 /* Robot */ + 5.669 /* Battery */ + 7 /* Bumpers */ // kg
        const val kRobotMomentOfInertia = 6.04808081 // kg m^2
        const val kRobotAngularDrag = 12.0 // N*m / (rad/sec)

        val kRobotWidth = 27.inch
        val kRobotLength = 32.inch
        val kIntakeLength = 10.5.inch
        val kBumperLength = 2.875.inch

        val kRobotStartX = (kRobotLength / 2.0) + kBumperLength

        val kExchangeZoneBottomY = 14.5.feet
        val kPortalZoneBottomY = (27 - (29.69 / 12.0)).feet

        val kRobotSideStartY = kPortalZoneBottomY - (kRobotWidth / 2.0) - kBumperLength
        val kRobotCenterStartY = kExchangeZoneBottomY - (kRobotWidth / 2.0) - kBumperLength

        val kFrontToIntake = Pose2d(-kIntakeLength, 0.meter, 0.degree)
        val kCenterToIntake = Pose2d(-(kRobotLength / 2.0) - kIntakeLength, 0.meter, 0.degree)
        val kCenterToFrontBumper = Pose2d(-(kRobotLength / 2.0) - kBumperLength, 0.meter, 0.degree)

        val kWheelRadius = 3.06.inch
        val kTrackWidth = 0.6858.meter

        const val kStaticFrictionVoltage = 1.8 // Volts
        const val kVDrive = 0.115 // Volts per radians per second
        const val kADrive = 0.0716 // Volts per radians per second per second

        private val transmission = DCMotorTransmission(
                1 / kVDrive,
                kWheelRadius.value.pow(2) * kRobotMass / (2.0 * kADrive),
                kStaticFrictionVoltage
        )

        val driveModel = DifferentialDrive(
                kRobotMass,
                kRobotMomentOfInertia,
                kRobotAngularDrag,
                kWheelRadius.value,
                kTrackWidth.value / 2.0,
                transmission,
                transmission
        )

        val MODEL = NativeUnitLengthModel(TICKS_PER_REVOLUTION.STU, kWheelRadius)

        const val kB = 0.9
        const val kZeta = 1.5
    }

    object Intake {
        const val VOLTAGE_COMPENSATION = 12.0
        const val FILTER_WINDOW_SAMPLE = 32
    }

    object LinearSlide {
        const val RAMP_RATE = 0.5

        const val PID_SLOT_INDEX = 0
        const val PROFILE_SLOT_INDEX = 0

        const val kF = 0.21765957446
        const val kP = 1.0

        const val MAX_VELOCITY = 3525
        const val MAX_ACCELERATION = 3525

        const val ALLOWABLE_CLOSED_LOOP_ERROR = 1000

        const val FORWARD_SOFT_LIMIT = 100000
        const val REVERSE_SOFT_LIMIT = -1000

        const val VOLTAGE_COMPENSATION = 12.0
        const val FILTER_WINDOW_SAMPLE = 32

        const val kGravity = 0.05
    }
}