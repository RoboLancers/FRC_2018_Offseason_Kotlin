package frc.team321.robot.utilities.enums

import edu.wpi.first.wpilibj.DoubleSolenoid

enum class IntakePivotState(val value: DoubleSolenoid.Value){
    UP(DoubleSolenoid.Value.kForward), DOWN(DoubleSolenoid.Value.kReverse)
}