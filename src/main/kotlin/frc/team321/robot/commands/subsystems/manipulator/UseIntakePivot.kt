package frc.team321.robot.commands.subsystems.manipulator

import edu.wpi.first.wpilibj.command.InstantCommand
import frc.team321.robot.subsystems.manipulator.IntakePivot
import frc.team321.robot.utilities.enums.IntakePivotState

class UseIntakePivot(private val intakePivotState: IntakePivotState): InstantCommand(){
    init {
        requires(IntakePivot)
    }

    override fun initialize() {
        IntakePivot.set(intakePivotState)
    }
}