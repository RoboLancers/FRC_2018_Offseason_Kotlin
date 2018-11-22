package frc.team321.robot.subsystems.drivetrain

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.command.Subsystem
import frc.team321.robot.RobotMap
import frc.team321.robot.commands.subsystems.drivetrain.UseDrive
import frc.team321.robot.utilities.enums.DrivetrainSide

@Suppress("unused")
object Drivetrain : Subsystem() {
    val leftTransmission = Transmission(DrivetrainSide.LEFT, RobotMap.Drivetrain.LEFT_MASTER, RobotMap.Drivetrain.LEFT_SLAVE1, RobotMap.Drivetrain.LEFT_SLAVE2)
    val rightTransmission = Transmission(DrivetrainSide.RIGHT, RobotMap.Drivetrain.RIGHT_MASTER, RobotMap.Drivetrain.RIGHT_SLAVE1, RobotMap.Drivetrain.RIGHT_SLAVE2)

    fun set(controlMode: ControlMode, leftValue: Double, rightValue: Double){
        leftTransmission.set(controlMode, leftValue)
        rightTransmission.set(controlMode, rightValue)
    }

    fun reset(){
        leftTransmission.reset()
        rightTransmission.reset()
    }

    override fun initDefaultCommand() {
        defaultCommand = UseDrive()
    }
}