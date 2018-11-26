package frc.team321.robot.utilities.motion.commands

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.command.Command
import frc.team321.robot.Constants
import frc.team321.robot.subsystems.drivetrain.Drivetrain
import frc.team321.robot.utilities.motion.Localization
import org.ghrobotics.lib.mathematics.twodim.control.RamseteController
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory
import org.ghrobotics.lib.mathematics.units.nativeunits.STUPer100ms
import org.ghrobotics.lib.mathematics.units.nativeunits.fromModel

@Suppress("unused")
class RamseteFollowerCommand(private val path: TimedTrajectory<Pose2dWithCurvature>, zeroPose: Boolean): Command(){
    private val follower = RamseteController(Constants.Drivetrain.driveModel, Constants.Drivetrain.kB, Constants.Drivetrain.kZeta)

    init {
        requires(Drivetrain)

        if (zeroPose) {
            Localization.reset(path.firstState.state.pose)
        }
    }

    override fun initialize() {
        follower.resetTrajectory(path)
    }

    override fun execute() {
        val currentPose = Localization.position

        val output = follower.getOutputFromDynamics(currentPose)

        updateDashboardValues()

        Drivetrain.set(
                ControlMode.Velocity,
                output.leftSetPoint.fromModel(Constants.Drivetrain.MODEL).STUPer100ms,
                output.rightSetPoint.fromModel(Constants.Drivetrain.MODEL).STUPer100ms,
                output.leftVoltage.value / 12.0,
                output.rightVoltage.value / 12.0
        )
    }

    override fun end() {
        Drivetrain.stop()
    }


    override fun isFinished(): Boolean {
        return follower.isFinished
    }

    private fun updateDashboardValues() {
        pathX = follower.referencePose.translation.x.feet
        pathY = follower.referencePose.translation.y.feet
        pathHdg = follower.referencePose.rotation.radian
    }

    companion object {
        var pathX = 0.0
            private set

        var pathY = 0.0
            private set

        var pathHdg = 0.0
            private set
    }
}