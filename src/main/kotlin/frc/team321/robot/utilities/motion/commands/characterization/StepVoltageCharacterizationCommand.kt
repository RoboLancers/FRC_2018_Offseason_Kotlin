package frc.team321.robot.utilities.motion.commands.characterization

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.command.Command
import frc.team321.robot.subsystems.drivetrain.Drivetrain
import org.ghrobotics.lib.mathematics.units.Length
import org.ghrobotics.lib.mathematics.units.millisecond
import org.ghrobotics.lib.subsystems.drive.characterization.CharacterizationData
import org.ghrobotics.lib.utils.DeltaTime


/**
 * Runs a step voltage test by setting a constant voltage to measure Ka
 *
 * @param wheelRadius Wheel radius
 * @param turnInPlace Whether the test should move forward for linear Ka or turn in place for angular Ka
 */
class StepVoltageCharacterizationCommand(
        private val wheelRadius: Length,
        private val turnInPlace: Boolean
) : Command(){

    private val data = ArrayList<CharacterizationData>()
    private val dtController = DeltaTime()

    init{
        requires(Drivetrain)
    }

    override fun initialize() {
        dtController.reset()
    }

    override fun execute() {
        val dt = dtController.updateTime(System.currentTimeMillis().millisecond)

        Drivetrain.set(ControlMode.PercentOutput, kStepVoltage / 12.0, kStepVoltage / 12.0 * if (turnInPlace) -1 else 1)

        val avgCompensatedVoltage =
                (Drivetrain.leftTransmission.master.motorOutputVoltage + Drivetrain.rightTransmission.master.motorOutputVoltage) / 2.0

        val avgSpd =
                (Drivetrain.leftTransmission.velocity + Drivetrain.rightTransmission.velocity).value /
                        2.0 / wheelRadius.value

        data.add(CharacterizationData(avgCompensatedVoltage, avgSpd, dt.second))
    }

    override fun isFinished(): Boolean {
        return false
    }

    override fun end() {
        println("ACCELERATION DATA")
        data.forEach { System.out.println(it.toCSV()) }
    }

    companion object {
        private const val kStepVoltage = 6.0
    }

}