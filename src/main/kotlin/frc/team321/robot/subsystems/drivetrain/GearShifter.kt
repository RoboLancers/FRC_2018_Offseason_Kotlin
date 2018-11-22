package frc.team321.robot.subsystems.drivetrain

import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj.command.Subsystem
import frc.team321.robot.RobotMap
import frc.team321.robot.utilities.enums.GearShifterGear

object GearShifter: Subsystem(){
    private val gearShifter = DoubleSolenoid(RobotMap.GearShifter.FORWARD, RobotMap.GearShifter.REVERSE)

    fun set(gearShifterGear: GearShifterGear){
        gearShifter.set(gearShifterGear.value)
    }

    fun set(value: DoubleSolenoid.Value){
        gearShifter.set(value)
    }

    override fun initDefaultCommand() {

    }
}