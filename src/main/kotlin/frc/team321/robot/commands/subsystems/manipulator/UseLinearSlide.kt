package frc.team321.robot.commands.subsystems.manipulator

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.command.Command
import frc.team321.robot.OI
import frc.team321.robot.subsystems.manipulator.LinearSlide
import net.jafama.FastMath

class UseLinearSlide: Command(){
    init {
        requires(LinearSlide)
    }

    override fun execute() {
        val joystickValue = OI.flightController.yAxisValue
        val encoderPosition = LinearSlide.encoderPosition

        val output = when {
            joystickValue > 0 -> -FastMath.tanh((encoderPosition - 100000)/5000.0)
            joystickValue < 0 -> FastMath.tanh(encoderPosition/5000.0)
            else -> 0.0
        }

        LinearSlide.set(ControlMode.PercentOutput, output * joystickValue)
    }

    override fun isFinished(): Boolean {
        return false
    }

}