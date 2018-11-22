package frc.team321.robot.commands.subsystems.manipulator

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.command.InstantCommand
import frc.team321.robot.subsystems.manipulator.Intake
import frc.team321.robot.utilities.enums.IntakePower

class UseIntake(private val intakePower: IntakePower): InstantCommand(){
    init {
        requires(Intake)
    }

    override fun initialize() {
        Intake.set(ControlMode.PercentOutput, intakePower.power)
    }
}