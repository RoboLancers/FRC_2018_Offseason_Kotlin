package frc.team321.robot.utilities.motion

import jaci.pathfinder.Trajectory

@Suppress("unused")
object Odometry{
    @Volatile
    @set:Synchronized
    @get:Synchronized
    var x: Double = 0.0

    @Volatile
    @set:Synchronized
    @get:Synchronized
    var y: Double = 0.0

    @Volatile
    @set:Synchronized
    @get:Synchronized
    var theta: Double = 0.0
        get() = field % (2 * Math.PI)

    @Volatile
    @set:Synchronized
    @get:Synchronized
    var currentEncoderPosition: Double = 0.0

    @Volatile
    @set:Synchronized
    @get:Synchronized
    var lastPosition: Double = 0.0

    @Volatile
    @set:Synchronized
    @get:Synchronized
    var deltaPosition: Double = 0.0

    @Synchronized
    fun setInitialOdometry(trajectory: Trajectory) {
        x = trajectory.get(0).x
        y = trajectory.get(0).y
        theta = trajectory.get(0).heading
    }

    override fun toString(): String {
        return "X Position: $x Y Position: $y Heading: $theta"
    }
}