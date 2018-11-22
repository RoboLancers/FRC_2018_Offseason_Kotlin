package frc.team321.robot.subsystems.manipulator

import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj.command.Subsystem
import frc.team321.robot.RobotMap
import frc.team321.robot.utilities.enums.IntakePivotState

object IntakePivot: Subsystem(){
    private val intakePivot = DoubleSolenoid(RobotMap.IntakePivot.FORWARD, RobotMap.IntakePivot.REVERSE)

    fun set(intakePivotState: IntakePivotState){
        intakePivot.set(intakePivotState.value)
    }

    fun set(value: DoubleSolenoid.Value){
        intakePivot.set(value)
    }

    override fun initDefaultCommand() {

    }
}