package frc.team321.robot.commands.subsystems.misc

import edu.wpi.first.wpilibj.command.Command
import frc.team321.robot.subsystems.misc.Pneumatic

class RegulateCompressor : Command() {
    init {
        requires(Pneumatic)
        isInterruptible = false
    }

    override fun execute() {
        Pneumatic.regulateCompressor()
    }

    override fun isFinished(): Boolean {
        return false
    }
}