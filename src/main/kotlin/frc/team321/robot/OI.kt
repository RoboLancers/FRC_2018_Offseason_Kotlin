package frc.team321.robot

import frc.team321.robot.commands.subsystems.manipulator.UseIntake
import frc.team321.robot.commands.subsystems.manipulator.UseIntakePivot
import frc.team321.robot.utilities.controllers.FlightController
import frc.team321.robot.utilities.controllers.XboxController
import frc.team321.robot.utilities.enums.IntakePivotState
import frc.team321.robot.utilities.enums.IntakePower

@Suppress("MemberVisibilityCanBePrivate")
object OI {
    val xboxController = XboxController(0)
    val flightController = FlightController(1)

    init {
        flightController.innerTop.whenPressed(UseIntakePivot(IntakePivotState.UP))
        flightController.innerTop.whenReleased(UseIntakePivot(IntakePivotState.DOWN))

        xboxController.leftBumper.whenPressed(UseIntake(IntakePower.INTAKE))
        xboxController.leftBumper.whenReleased(UseIntake(IntakePower.PASSIVE))

        xboxController.rightBumper.whenPressed(UseIntake(IntakePower.OUTTAKE))
        xboxController.rightBumper.whenReleased(UseIntake(IntakePower.STOP))

        flightController.trigger.whenPressed(UseIntake(IntakePower.OUTTAKE_SLOW))
        flightController.trigger.whenReleased(UseIntake(IntakePower.STOP))
    }
}