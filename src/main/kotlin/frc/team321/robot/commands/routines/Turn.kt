package frc.team321.robot.commands.routines

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.command.Command
import frc.team321.robot.Constants
import frc.team321.robot.subsystems.drivetrain.Drivetrain
import frc.team321.robot.subsystems.misc.Sensors
import frc.team321.robot.utilities.motion.LancerPID
import kotlin.math.absoluteValue

class Turn(val angle: Double): Command(){
    private val lancerPID = LancerPID(Constants.Drivetrain.TURN_KP, Constants.Drivetrain.TURN_KI, Constants.Drivetrain.TURN_KD)

    override fun initialize() {
        lancerPID.setSetpoint(angle)
    }

    override fun execute() {
        val output = lancerPID.getOutput(Sensors.angle)
        Drivetrain.set(ControlMode.PercentOutput, -output, output)
    }

    override fun isFinished(): Boolean {
        return (Sensors.angle - angle).absoluteValue < 0.1
    }
}