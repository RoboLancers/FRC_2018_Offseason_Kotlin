package frc.team321.robot.commands.subsystems.drivetrain

import edu.wpi.first.wpilibj.command.InstantCommand
import frc.team321.robot.subsystems.drivetrain.GearShifter
import frc.team321.robot.utilities.enums.GearShifterGear

@Suppress("unused")
class UseGearShifter(private val gearShifterGear: GearShifterGear): InstantCommand(){
    init {
        requires(GearShifter)
    }

    override fun initialize() {
        GearShifter.set(gearShifterGear)
    }
}