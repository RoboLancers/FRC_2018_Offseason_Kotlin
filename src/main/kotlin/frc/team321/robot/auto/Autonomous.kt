package frc.team321.robot.auto

import edu.wpi.first.wpilibj.command.CommandGroup
import frc.team321.robot.NetworkInterface
import frc.team321.robot.auto.routines.CharacterizationRoutine
import frc.team321.robot.commands.routines.DoNothingAndReset
import openrio.powerup.MatchData
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d

object Autonomous{
    object Config {
        val startingPosition = { NetworkInterface.startingPositionChooser.selected }
        val switchSide = { MatchData.getOwnedSide(MatchData.GameFeature.SWITCH_NEAR) }
        val scaleSide = { MatchData.getOwnedSide(MatchData.GameFeature.SCALE) }
        val switchAutoMode = { NetworkInterface.startingPositionChooser.selected }
        val scaleAutoMode = { NetworkInterface.sideModeChooser.selected }
        val autoMode = { NetworkInterface.autoModeChooser.selected }
    }

    var routine: CommandGroup = if(Config.autoMode == AutoMode.REAL){
        DoNothingAndReset()
    }else{
        CharacterizationRoutine()
    }
}

enum class AutoMode{
    REAL, CHARACTERIZATION
}

enum class StartingPositions(
        val pose: Pose2d,
        private val matchSide: MatchData.OwnedSide
) {
    LEFT(Trajectories.kSideStart, MatchData.OwnedSide.LEFT),
    CENTER(Trajectories.kCenterStart, MatchData.OwnedSide.UNKNOWN),
    RIGHT(Trajectories.kSideStart.mirror, MatchData.OwnedSide.RIGHT);

    fun isSameSide(side: MatchData.OwnedSide) = matchSide == side
}

enum class SideMode{
    SWITCH, SCALE
}