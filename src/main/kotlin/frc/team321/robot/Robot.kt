package frc.team321.robot

import com.ctre.phoenix.motorcontrol.NeutralMode
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.command.Scheduler
import frc.team321.robot.auto.Autonomous
import frc.team321.robot.subsystems.drivetrain.Drivetrain
import frc.team321.robot.subsystems.drivetrain.GearShifter
import frc.team321.robot.subsystems.manipulator.Intake
import frc.team321.robot.subsystems.manipulator.IntakePivot
import frc.team321.robot.subsystems.manipulator.LinearSlide
import frc.team321.robot.subsystems.misc.Camera
import frc.team321.robot.subsystems.misc.Pneumatic
import frc.team321.robot.subsystems.misc.Sensors
import frc.team321.robot.utilities.enums.GearShifterGear
import frc.team321.robot.utilities.enums.IntakePivotState
import frc.team321.robot.utilities.motion.Localization

class Robot : TimedRobot() {
    override fun robotInit() {
        Drivetrain
        GearShifter

        Intake
        IntakePivot
        LinearSlide

        Pneumatic
        Camera.start()

        OI
        Localization
        Autonomous

        GearShifter.set(GearShifterGear.HIGH)
        Drivetrain.setNeutralMode(NeutralMode.Brake)
    }

    override fun autonomousInit() {
        Drivetrain.setRamping(false)
        Sensors.navX.reset()
        IntakePivot.set(IntakePivotState.UP)

        Autonomous.routine.start()
    }

    override fun teleopInit() {
        Drivetrain.setRamping(true)
        IntakePivot.set(IntakePivotState.DOWN)
    }

    override fun testInit() {
        IntakePivot.set(IntakePivotState.UP)
    }

    override fun robotPeriodic() {
        NetworkInterface.updateDashboardValues()
        Scheduler.getInstance().run()
    }
}