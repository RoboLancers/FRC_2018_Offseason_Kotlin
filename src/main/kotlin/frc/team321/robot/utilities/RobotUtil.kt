package frc.team321.robot.utilities

import frc.team321.robot.Constants

@Suppress("unused")
class RobotUtil{
    companion object {
        fun applyDeadband(value: Double, deadband: Double = Constants.JOYSTICK_DEADBAND) = if (value in -deadband..deadband) {
                0.0
            } else {
                value
            }

        fun feetToEncoderTick(feet: Double): Double {
            return (feet * Constants.Drivetrain.TICKS_PER_REVOLUTION) / (Math.PI * Constants.Drivetrain.WHEEL_DIAMETER)
        }

        fun encoderTickToFeet(ticks: Double): Double {
            return (ticks / Constants.Drivetrain.TICKS_PER_REVOLUTION) * (Math.PI * Constants.Drivetrain.WHEEL_DIAMETER)
        }
    }
}