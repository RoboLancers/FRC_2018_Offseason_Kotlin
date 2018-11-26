package frc.team321.robot.utilities.motion

import jaci.pathfinder.Pathfinder
import jaci.pathfinder.Trajectory
import jaci.pathfinder.Trajectory.Segment

import java.io.File

class TrajectoryUtil {
    companion object {
        fun reversePath(original: Trajectory): Trajectory {
            val segments = original.segments.reversedArray()

            val distance = segments[0].position

            return Trajectory(segments.map { Segment(it.dt, it.x, it.y, distance - it.position, -it.velocity, -it.acceleration, -it.jerk, it.heading) }.toTypedArray())
        }

        fun getTrajectoryFromName(trajectoryName: String): Trajectory? {
            return Pathfinder.readFromFile(File("/home/lvuser/trajectories/" + trajectoryName + "_source_detailed.traj"))
        }
    }
}