package frc.team321.robot.commands.routines

import edu.wpi.first.wpilibj.command.CommandGroup
import frc.team321.robot.subsystems.drivetrain.Drivetrain
import frc.team321.robot.subsystems.misc.Sensors

class DoNothingAndReset: CommandGroup(){
    init{
        Drivetrain.reset()
        Sensors.navX.reset()
    }
}