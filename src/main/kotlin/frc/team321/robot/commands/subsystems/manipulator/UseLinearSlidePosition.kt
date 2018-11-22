package frc.team321.robot.commands.subsystems.manipulator

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.command.Command
import frc.team321.robot.Constants
import frc.team321.robot.OI
import frc.team321.robot.subsystems.manipulator.LinearSlide
import frc.team321.robot.utilities.enums.LinearSlidePosition

@Suppress("unused")
class UseLinearSlidePosition(private val linearSlidePosition: LinearSlidePosition): Command(){
    override fun initialize() {
        LinearSlide.setRamping(false)
        LinearSlide.set(ControlMode.MotionMagic, linearSlidePosition.position)
    }

    override fun end() {
        LinearSlide.setRamping(true)
    }

    override fun isFinished(): Boolean {
        return (Math.abs(LinearSlide.master.activeTrajectoryPosition - linearSlidePosition.position) < Constants.LinearSlide.ALLOWABLE_CLOSED_LOOP_ERROR)
                || Math.abs(OI.flightController.yAxisValue) > 0
    }
}