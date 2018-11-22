package frc.team321.robot

@Suppress("MemberVisibilityCanBePrivate", "unused")
object Constants {

    const val TIMEOUT = 0

    const val JOYSTICK_DEADBAND = 0.1

    object Drivetrain {
        const val PID_SLOT_INDEX = 0
        const val PROFILE_SLOT_INDEX = 0

        const val LEFT_KF = 1.21
        const val RIGHT_KF = 1.17

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