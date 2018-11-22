package frc.team321.robot.subsystems.misc

import frc.team321.robot.RobotMap
import frc.team321.robot.commands.subsystems.misc.RegulateCompressor

import edu.wpi.first.wpilibj.Compressor
import edu.wpi.first.wpilibj.command.Subsystem

@Suppress("unused")
object Pneumatic: Subsystem() {
    private val compressor = Compressor(RobotMap.COMPRESSOR)

    private val isCompressorSafeToUse: Boolean
        get() = !(compressor.compressorCurrentTooHighFault && !compressor.compressorCurrentTooHighStickyFault ||
                compressor.compressorNotConnectedFault && !compressor.compressorNotConnectedStickyFault ||
                compressor.compressorShortedFault && !compressor.compressorShortedStickyFault)

    override fun initDefaultCommand() {
        defaultCommand = RegulateCompressor()
    }

    fun regulateCompressor() {
        if (!compressor.pressureSwitchValue && !compressor.enabled() && isCompressorSafeToUse) {
            compressor.start()
        } else if (compressor.pressureSwitchValue && compressor.enabled() || !isCompressorSafeToUse) {
            compressor.stop()
        }
    }

    fun stopCompressor() {
        compressor.stop()
    }
}