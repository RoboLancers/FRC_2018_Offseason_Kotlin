package frc.team321.robot.auto

import frc.team321.robot.Constants
import frc.team321.robot.Constants.Drivetrain.kCenterToFrontBumper
import frc.team321.robot.Constants.Drivetrain.kCenterToIntake
import frc.team321.robot.Constants.Drivetrain.kRobotCenterStartY
import frc.team321.robot.Constants.Drivetrain.kRobotSideStartY
import frc.team321.robot.Constants.Drivetrain.kRobotStartX
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature
import org.ghrobotics.lib.mathematics.twodim.trajectory.DefaultTrajectoryGenerator
import org.ghrobotics.lib.mathematics.twodim.trajectory.constraints.CentripetalAccelerationConstraint
import org.ghrobotics.lib.mathematics.twodim.trajectory.constraints.DifferentialDriveDynamicsConstraint
import org.ghrobotics.lib.mathematics.twodim.trajectory.constraints.TimingConstraint
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory
import org.ghrobotics.lib.mathematics.units.degree
import org.ghrobotics.lib.mathematics.units.derivedunits.*
import org.ghrobotics.lib.mathematics.units.feet
import org.ghrobotics.lib.mathematics.units.meter

@Suppress("unused", "MemberVisibilityCanBePrivate")
object Trajectories {
    // Constants in Feet Per Second
    private val kMaxVelocity = 8.0.feet.velocity
    private val kMaxAcceleration = 4.0.feet.acceleration
    private val kMaxCentripetalAcceleration = 4.5.feet.acceleration

    // Constraints
    private val kConstraints = listOf(
            CentripetalAccelerationConstraint(kMaxCentripetalAcceleration),
            DifferentialDriveDynamicsConstraint(Constants.Drivetrain.driveModel, 10.0.volt)
    )

    // Field Relative Constants
    internal val kSideStart = Pose2d(kRobotStartX, kRobotSideStartY, 0.degree)
    internal val kCenterStart = Pose2d(kRobotStartX, kRobotCenterStartY, 0.degree)

    private val kNearScaleEmpty = Pose2d(23.95.feet, 20.2.feet, (-20).degree)
    private val kNearScaleFull = Pose2d(23.95.feet, 20.feet, (-10).degree)
    private val kNearScaleFullInner = Pose2d(24.3.feet, 20.feet, (-10).degree)

    private val kNearCube1 = Pose2d(16.5.feet, 19.2.feet, 190.degree)
    private val kNearCube2 = Pose2d(17.4.feet, 15.feet, 245.degree)
    private val kNearCube3 = Pose2d(17.6.feet, 14.5.feet, 245.degree)

    private val kNearCube1Adjusted = kNearCube1 + kCenterToIntake
    private val kNearCube2Adjusted = kNearCube2 + kCenterToIntake
    private val kNearCube3Adjusted = kNearCube3 + kCenterToIntake

    private val kFarCube1 = Pose2d(16.5.feet, 20.feet, 195.degree)
    private val kFarCube1Adjusted = kFarCube1 + kCenterToIntake

    private val kSwitchLeft = Pose2d(11.9.feet, 18.5.feet, 0.degree)
    private val kSwitchRight = Pose2d(11.9.feet, 8.5.feet, 0.degree)

    private val kSwitchLeftAdjusted = kSwitchLeft + kCenterToFrontBumper
    private val kSwitchRightAdjusted = kSwitchRight + kCenterToFrontBumper

    private val kFrontPyramidCube = Pose2d(10.25.feet, 13.5.feet, 0.degree)
    private val kFrontPyramidCubeAdjusted = kFrontPyramidCube + kCenterToIntake

    // FastTrajectories
    val leftStartToNearScale = waypoints(
            kSideStart,
            kSideStart + Pose2d(10.feet, 0.feet, 0.degree),
            kNearScaleEmpty
    ).generateTrajectory("Left Start to Near Scale", reversed = false)

    val leftStartToFarScale = waypoints(
            kSideStart,
            kSideStart + Pose2d((-13).feet, 0.feet, 0.degree),
            kSideStart + Pose2d((-18.9).feet, 5.feet, (-90).degree),
            kSideStart + Pose2d((-18.9).feet, 12.feet, (-90).degree),
            kNearScaleEmpty.mirror
    ).generateTrajectory("Left Start to Far Scale", reversed = true)

    val scaleToCube1 = waypoints(
            kNearScaleEmpty,
            kNearCube1Adjusted
    ).generateTrajectory("Scale to Cube 1", reversed = false)

    val cube1ToScale = waypoints(
            kNearCube1Adjusted,
            kNearScaleFull
    ).generateTrajectory("Cube 1 to Scale", reversed = true)

    val scaleToCube2 = waypoints(
            kNearScaleFull,
            kNearCube2Adjusted
    ).generateTrajectory("Scale to Cube 2", reversed = false)

    val cube2ToScale = waypoints(
            kNearCube2Adjusted,
            kNearScaleFullInner
    ).generateTrajectory("Cube 2 to Scale", reversed = true)

    val scaleToCube3 = waypoints(
            kNearScaleFull,
            kNearCube3Adjusted
    ).generateTrajectory("Scale to Cube 3", reversed = false)

    val scaleToFar1 = waypoints(
            kNearScaleEmpty,
            kFarCube1Adjusted
    ).generateTrajectory("Scale to Far 1", reversed = false)

    val far1ToScale = waypoints(
            kFarCube1Adjusted,
            kNearScaleFullInner
    ).generateTrajectory("Far 1 to Scale", reversed = true)

    val cube3ToScale = waypoints(
            kNearCube3Adjusted,
            kNearScaleFullInner
    ).generateTrajectory("Cube 3 to Scale", reversed = true)

    val centerStartToLeftSwitch = waypoints(
            kCenterStart,
            kSwitchLeftAdjusted
    ).generateTrajectory("Center Start to Left Switch", reversed = false)

    val centerStartToRightSwitch = waypoints(
            kCenterStart,
            kSwitchRightAdjusted
    ).generateTrajectory("Center Start to Right Switch", reversed = false)

    val switchToCenter = waypoints(
            kSwitchLeftAdjusted,
            kFrontPyramidCubeAdjusted + Pose2d((-4).feet, 0.feet, 0.degree)
    ).generateTrajectory("Switch to Center", reversed = true)

    val centerToPyramid = waypoints(
            kFrontPyramidCubeAdjusted + Pose2d((-4.0).feet, 0.feet, 0.degree),
            kFrontPyramidCubeAdjusted
    ).generateTrajectory("Center to Pyramid", reversed = false)

    val pyramidToCenter = waypoints(
            kFrontPyramidCubeAdjusted,
            kFrontPyramidCubeAdjusted + Pose2d((-4).feet, 0.feet, 0.degree)
    ).generateTrajectory("Pyramid to Center", reversed = true)

    val centerToSwitch = waypoints(
            kFrontPyramidCubeAdjusted + Pose2d((-4).feet, 0.feet, 0.degree),
            kSwitchLeftAdjusted
    ).generateTrajectory("Center to Switch", reversed = false)

    val pyramidToScale = waypoints(
            kFrontPyramidCubeAdjusted,
            kFrontPyramidCubeAdjusted + Pose2d(2.feet, 9.feet, 180.degree),
            kFrontPyramidCubeAdjusted + Pose2d(7.feet, 9.feet, 180.degree),
            kNearScaleEmpty
    ).generateTrajectory(
            "Pyramid to Scale",
            reversed = true,
            maxVelocity = 4.feet.velocity,
            maxAcceleration = 3.feet.acceleration,
            constraints = listOf(CentripetalAccelerationConstraint(3.0.feet.acceleration))
    )

    val baseline = waypoints(
            kSideStart,
            kSideStart + Pose2d((-8.0).feet, 0.0.feet, 0.degree)
    ).generateTrajectory("Baseline", reversed = true)

    private fun waypoints(vararg waypoints: Pose2d) = listOf(*waypoints)

    private fun List<Pose2d>.generateTrajectory(
            name: String,
            reversed: Boolean,
            maxVelocity: LinearVelocity = kMaxVelocity,
            maxAcceleration: LinearAcceleration = kMaxAcceleration,
            constraints: List<TimingConstraint<Pose2dWithCurvature>> = kConstraints
    ): TimedTrajectory<Pose2dWithCurvature> {
        println("Generating $name")
        return DefaultTrajectoryGenerator.generateTrajectory(
                reversed = reversed,
                wayPoints = this,
                constraints = constraints,
                startVelocity = 0.meter.velocity,
                endVelocity = 0.meter.velocity,
                maxVelocity = maxVelocity,
                maxAcceleration = maxAcceleration
        )
    }

}
