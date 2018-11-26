package frc.team321.robot.auto.routines

import edu.wpi.first.wpilibj.command.CommandGroup
import edu.wpi.first.wpilibj.command.PrintCommand
import frc.team321.robot.Constants
import frc.team321.robot.utilities.motion.commands.characterization.QuasistaticCharacterizationCommand
import frc.team321.robot.utilities.motion.commands.characterization.StepVoltageCharacterizationCommand

class CharacterizationRoutine: CommandGroup(){
    init{
        addSequential(PrintCommand("Running QuasistaticCharacterizationCommand without turning"))
        addSequential(QuasistaticCharacterizationCommand(Constants.Drivetrain.kWheelRadius, false))
        addSequential(PrintCommand("Running StepVoltageCharacterizationCommand without turning"))
        addSequential(StepVoltageCharacterizationCommand(Constants.Drivetrain.kWheelRadius, false))
        addSequential(PrintCommand("Running QuasistaticCharacterizationCommand with turning"))
        addSequential(QuasistaticCharacterizationCommand(Constants.Drivetrain.kWheelRadius, true))
        addSequential(PrintCommand("Running StepVoltageCharacterizationCommand without turning"))
        addSequential(StepVoltageCharacterizationCommand(Constants.Drivetrain.kWheelRadius, true))
    }
}