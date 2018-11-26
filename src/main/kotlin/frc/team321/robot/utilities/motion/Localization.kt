package frc.team321.robot.utilities.motion

import edu.wpi.first.wpilibj.Notifier
import frc.team321.robot.Constants
import frc.team321.robot.subsystems.drivetrain.Drivetrain
import frc.team321.robot.subsystems.misc.Sensors
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d
import org.ghrobotics.lib.mathematics.units.degree
import org.ghrobotics.lib.mathematics.units.feet
import org.ghrobotics.lib.mathematics.units.radian
import kotlin.math.cos
import kotlin.math.sin

@Suppress("MemberVisibilityCanBePrivate")
object Localization{
    private val lock = Any()

    private var prevL = 0.0
    private var prevR = 0.0

    var position = Pose2d(0.0.feet, 0.0.feet, 0.0.degree)
        @Synchronized get
        private set

    init {
        reset()
        Notifier(::run).startPeriodic(0.01)
    }

    fun reset(resetPose: Pose2d = Pose2d(0.0.feet, 0.0.feet, 0.0.degree)) {
        synchronized(lock) {
            position = resetPose
            prevL = Drivetrain.leftTransmission.position.toModel(Constants.Drivetrain.MODEL).feet
            prevR = Drivetrain.rightTransmission.position.toModel(Constants.Drivetrain.MODEL).feet
        }
    }

    private fun run() {
        synchronized(lock) {
            val currentL = Drivetrain.leftTransmission.position.toModel(Constants.Drivetrain.MODEL).feet
            val currentR = Drivetrain.rightTransmission.position.toModel(Constants.Drivetrain.MODEL).feet
            val heading = Math.toRadians(Sensors.angle)

            val deltaL = currentL - prevL
            val deltaR = currentR - prevR

            val dist = (deltaL + deltaR) / 2.0

            val x = dist * cos(heading)
            val y = dist * sin(heading)

            position = Pose2d(
                    Translation2d(
                            position.translation.x + x.feet,
                            position.translation.y + y.feet
                    ),
                    heading.radian
            )

            prevL = currentL
            prevR = currentR
        }
    }
}