package frc.team321.robot.utilities.enums

import edu.wpi.first.wpilibj.DoubleSolenoid

enum class GearShifterGear(val value: DoubleSolenoid.Value) {
    LOW(DoubleSolenoid.Value.kReverse), HIGH(DoubleSolenoid.Value.kForward)
}