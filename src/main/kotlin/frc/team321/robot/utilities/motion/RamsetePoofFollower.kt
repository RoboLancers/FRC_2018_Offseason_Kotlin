package frc.team321.robot.utilities.motion

import com.team254.lib.physics.DifferentialDrive
import frc.team321.robot.Constants
import org.ghrobotics.lib.mathematics.epsilonEquals
import org.ghrobotics.lib.mathematics.twodim.control.TrajectoryFollower
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory
import org.ghrobotics.lib.mathematics.units.*
import org.ghrobotics.lib.mathematics.units.derivedunits.feetPerSecond
import org.ghrobotics.lib.mathematics.units.derivedunits.velocity
import org.ghrobotics.lib.mathematics.units.derivedunits.volt
import org.ghrobotics.lib.mathematics.units.nativeunits.STUPer100ms
import org.ghrobotics.lib.mathematics.units.nativeunits.fromModel
import org.ghrobotics.lib.utils.DeltaTime
import kotlin.math.pow
import kotlin.math.sin
import kotlin.math.sqrt

// https://www.dis.uniroma1.it/~labrob/pub/papers/Ramsete01.pdf
// Equation 5.12
@Suppress("MemberVisibilityCanBePrivate", "unused")
open class RamseteController(var drive: DifferentialDrive, trajectory: TimedTrajectory<Pose2dWithCurvature>, private val b: Double = Constants.Drivetrain.kB, private val zeta: Double = Constants.Drivetrain.kZeta) {

    // Store the previous velocity for acceleration calculations.
    private var previousVelocity = DifferentialDrive.ChassisState(0.0, 0.0)

    // Current reference point, pose, and variable to store if the iterator has finished.
    val iterator = trajectory.iterator()

    val referencePoint
        get() = iterator.currentState

    val referencePose
        get() = referencePoint.state.state.pose

    val isFinished
        get() = iterator.isDone

    var deltaTimeController = DeltaTime()

    init {
        if (zeta !in 0.0..1.0) {
            throw IllegalArgumentException("Zeta must be in (0, 1)")
        }

        if (b <= 0) {
            throw IllegalArgumentException("b must be > 0")
        }

        deltaTimeController.reset()
    }

    fun calculateChassisVelocity(robotPose: Pose2d): DifferentialDrive.ChassisState {
        // Calculate goal in robot's coordinates
        val error = referencePose inFrameOfReferenceOf robotPose

        // Get reference linear and angular velocities
        val vd = referencePoint.state.velocity.value
        val wd = vd * referencePoint.state.state.curvature.curvature.value

        // Compute gain
        val k1 = 2 * zeta * sqrt(wd * wd + b * vd * vd)

        // Get angular error in bounded radians
        val angleError = error.rotation.radian

        return DifferentialDrive.ChassisState(
                linear = vd * error.rotation.cos + k1 * error.translation.x.value,
                angular = wd + b * vd * sinc(angleError) * error.translation.y.value + k1 * angleError
        )
    }

    fun update(robotPose: Pose2d, currentTime: Time = System.nanoTime().nanosecond): TrajectoryFollower.Output {
        val dt = deltaTimeController.updateTime(currentTime)

        val chassisVelocity = calculateChassisVelocity(robotPose)
        val chassisAcceleration = if (dt == 0.0.second) {
            DifferentialDrive.ChassisState(0.0, 0.0)
        } else {
            (chassisVelocity - previousVelocity) / dt.second
        }

        val dynamics = drive.solveInverseDynamics(chassisVelocity, chassisAcceleration)

        previousVelocity = dynamics.chassisVelocity

        iterator.advance(dt)

        return outputFromWheelStates(drive, dynamics.wheelVelocity, dynamics.voltage)
    }

    fun k1(vd: Double, wd: Double): Double {
        return 2 * zeta * sqrt(wd.pow(2) + b * vd.pow(2))
    }

    companion object {
        private fun sinc(theta: Double): Double {
            return if (theta epsilonEquals 0.0) {
                1.0 // return the limit in cases where it approaches 0
            } else {
                sin(theta) / theta
            }
        }

        private fun outputFromWheelStates(
                drive: DifferentialDrive,
                setPoint: DifferentialDrive.WheelState,
                voltages: DifferentialDrive.WheelState
        ) = TrajectoryFollower.Output(
                leftSetPoint = (setPoint.left * drive.wheelRadius).meter.velocity,
                rightSetPoint = (setPoint.right * drive.wheelRadius).meter.velocity,
                leftVoltage = voltages.left.volt,
                rightVoltage = voltages.right.volt
        )
    }
}