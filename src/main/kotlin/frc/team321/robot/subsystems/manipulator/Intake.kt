package frc.team321.robot.subsystems.manipulator

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.NeutralMode
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import edu.wpi.first.wpilibj.command.Subsystem
import frc.team321.robot.Constants
import frc.team321.robot.RobotMap

object Intake: Subsystem(){
    private val intakeLeft = TalonSRX(RobotMap.Intake.LEFT)
    private val intakeRight = TalonSRX(RobotMap.Intake.RIGHT)

    private val talons = arrayOf(intakeLeft, intakeRight)

    init {
        for(talon in talons){
            talon.setNeutralMode(NeutralMode.Brake)

            talon.configVoltageCompSaturation(Constants.Intake.VOLTAGE_COMPENSATION, Constants.TIMEOUT)
            talon.configVoltageMeasurementFilter(Constants.Intake.FILTER_WINDOW_SAMPLE, Constants.TIMEOUT)
            talon.enableVoltageCompensation(true)
        }
    }

    fun set(controlMode: ControlMode, leftValue: Double, rightValue: Double){
        intakeLeft.set(controlMode, leftValue)
        intakeRight.set(controlMode, rightValue)
    }

    fun set(controlMode: ControlMode, value: Double){
        set(controlMode, value, value)
    }

    override fun initDefaultCommand() {

    }
}