package frc.team321.robot

import edu.wpi.first.wpilibj.IterativeRobot
import edu.wpi.first.wpilibj.Notifier
import frc.team321.robot.subsystems.drivetrain.Drivetrain
import frc.team321.robot.subsystems.drivetrain.GearShifter
import frc.team321.robot.subsystems.manipulator.Intake
import frc.team321.robot.subsystems.manipulator.IntakePivot
import frc.team321.robot.subsystems.manipulator.LinearSlide
import frc.team321.robot.subsystems.misc.Camera
import frc.team321.robot.subsystems.misc.Pneumatic
import frc.team321.robot.subsystems.misc.Sensors
import frc.team321.robot.utilities.RobotUtil
import frc.team321.robot.utilities.motion.Odometry

class Robot : IterativeRobot() {
    override fun robotInit() {
        Drivetrain
        GearShifter

        Intake
        IntakePivot
        LinearSlide

        Pneumatic
        Camera.start()

        OI

        Notifier {
            Odometry.currentEncoderPosition = (Drivetrain.leftTransmission.encoderCount + Drivetrain.rightTransmission.encoderCount) / 2.0
            Odometry.deltaPosition = RobotUtil.encoderTickToFeet(Odometry.currentEncoderPosition - Odometry.lastPosition)
            Odometry.theta = Math.toRadians(Sensors.angle)

            Odometry.x += Math.cos(Odometry.theta) * Odometry.deltaPosition
            Odometry.y += Math.sin(Odometry.theta) * Odometry.deltaPosition

            Odometry.lastPosition = Odometry.currentEncoderPosition
        }.startPeriodic(0.01)
    }

    override fun disabledInit() {}

    override fun autonomousInit() {}

    override fun teleopInit() {}

    override fun robotPeriodic() {}

    override fun disabledPeriodic() {}

    override fun autonomousPeriodic() {}

    override fun teleopPeriodic() {}
}