package frc.team321.robot

import edu.wpi.first.networktables.NetworkTable
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Notifier
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import frc.team321.robot.auto.AutoMode
import frc.team321.robot.auto.SideMode
import frc.team321.robot.auto.StartingPositions
import frc.team321.robot.utilities.motion.Localization
import frc.team321.robot.utilities.motion.commands.RamseteFollowerCommand
import org.ghrobotics.lib.wrappers.networktables.get

@Suppress("MemberVisibilityCanBePrivate")
object NetworkInterface{
    val startingPositionChooser = SendableChooser<StartingPositions>()
    val sideModeChooser = SendableChooser<SideMode>()
    val autoModeChooser = SendableChooser<AutoMode>()

    private val INSTANCE: NetworkTable = NetworkTableInstance.getDefault().getTable("Live Dashboard")

    private val robotX = INSTANCE["Robot X"]
    private val robotY = INSTANCE["Robot Y"]
    private val robotHdg = INSTANCE["Robot Heading"]

    private val pathX = INSTANCE["Path X"]
    private val pathY = INSTANCE["Path Y"]
    private val pathHdg = INSTANCE["Path Heading"]

    private val isEnabled = INSTANCE["Is Enabled"]

    private val notifier: Notifier

    init {
        AutoMode.values()
                .forEach { autoModeChooser.addObject(it.name.toLowerCase().capitalize(), it) }

        StartingPositions.values()
                .forEach { startingPositionChooser.addObject(it.name.toLowerCase().capitalize(), it) }

        SideMode.values()
                .forEach { sideModeChooser.addObject(it.name.toLowerCase().capitalize(), it) }

        notifier = Notifier {
            val pose = Localization.position
            robotX.setDouble(pose.translation.x.feet)
            robotY.setDouble(pose.translation.y.feet)
            robotHdg.setDouble(pose.rotation.radian)

            pathX.setDouble(RamseteFollowerCommand.pathX)
            pathY.setDouble(RamseteFollowerCommand.pathY)
            pathHdg.setDouble(RamseteFollowerCommand.pathHdg)

            isEnabled.setBoolean(DriverStation.getInstance().isEnabled)
        }

        notifier.startPeriodic(0.02)
    }

    fun updateDashboardValues(){

    }
}
