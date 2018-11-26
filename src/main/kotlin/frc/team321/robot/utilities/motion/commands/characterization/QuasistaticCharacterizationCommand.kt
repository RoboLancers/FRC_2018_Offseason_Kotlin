package frc.team321.robot.utilities.motion.commands.characterization

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.command.Command
import frc.team321.robot.subsystems.drivetrain.Drivetrain
import org.ghrobotics.lib.mathematics.units.Length
import org.ghrobotics.lib.mathematics.units.millisecond
import org.ghrobotics.lib.subsystems.drive.characterization.CharacterizationData
import org.ghrobotics.lib.utils.DeltaTime

/**
 * Runs a quasistatic test by ramping voltage slowly to measure Kv.
 *
 * @param wheelRadius Wheel radius
 * @param turnInPlace Whether the test should move forward for linear Kv or turn in place for angular Kv
 */
class QuasistaticCharacterizationCommand(
        private val wheelRadius: Length,
        private val turnInPlace: Boolean
) : Command() {

    private val data = ArrayList<CharacterizationData>()
    private val dtController = DeltaTime()

    private var startTime = 0.millisecond
    private var commandedVoltage = 0.0 // V

    init{
        requires(Drivetrain)
    }

    override fun initialize() {
        startTime = System.currentTimeMillis().millisecond
        dtController.reset()
    }

    override fun execute() {
        commandedVoltage = kRampRate * (System.currentTimeMillis().millisecond - startTime).second
        val dt = dtController.updateTime(System.currentTimeMillis().millisecond)

        Drivetrain.set(ControlMode.PercentOutput, commandedVoltage / 12.0, commandedVoltage / 12.0 * if (turnInPlace) -1 else 1)

        val avgCompensatedVoltage =
                (Drivetrain.leftTransmission.master.motorOutputVoltage + Drivetrain.rightTransmission.master.motorOutputVoltage) / 2.0

        val avgSpd =
                (Drivetrain.leftTransmission.velocity + Drivetrain.rightTransmission.velocity).value /
                        2.0 / wheelRadius.value

        data.add(CharacterizationData(avgCompensatedVoltage, avgSpd, dt.second))
    }

    override fun isFinished(): Boolean {
        return commandedVoltage >= kMaxVoltage
    }

    override fun end() {
        println("VELOCITY DATA")
        data.forEach { System.out.println(it.toCSV()) }
    }

    companion object {
        private const val kRampRate = 0.15 // V per sec
        private const val kMaxVoltage = 4.0 // V
    }

}