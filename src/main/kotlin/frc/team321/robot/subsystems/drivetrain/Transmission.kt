package frc.team321.robot.subsystems.drivetrain

import com.ctre.phoenix.motorcontrol.*
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX
import frc.team321.robot.Constants
import frc.team321.robot.utilities.enums.DrivetrainSide
import org.ghrobotics.lib.mathematics.units.derivedunits.LinearVelocity
import org.ghrobotics.lib.mathematics.units.derivedunits.velocity
import org.ghrobotics.lib.mathematics.units.feet
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnit
import org.ghrobotics.lib.mathematics.units.nativeunits.STU

@Suppress("unused", "MemberVisibilityCanBePrivate")
class Transmission(drivetrainSide: DrivetrainSide, vararg ports: Int) {
    val master: WPI_TalonSRX = WPI_TalonSRX(ports[0])
    private val slave1: TalonSRX = TalonSRX(ports[1])
    private val slave2: TalonSRX = TalonSRX(ports[2])

    private val talons = arrayOf(master, slave1, slave2)

    val encoderCount: Int
        get() = master.getSelectedSensorPosition(Constants.Drivetrain.PID_SLOT_INDEX)

    val encoderVelocity: Int
        get() = master.getSelectedSensorVelocity(Constants.Drivetrain.PID_SLOT_INDEX)

    val position: NativeUnit
        get() = encoderCount.STU

    val velocity: LinearVelocity
        get() = encoderVelocity.feet.velocity

    init {
        slave1.follow(master)
        slave2.follow(master)

        master.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, Constants.Drivetrain.PID_SLOT_INDEX, Constants.TIMEOUT)

        if (drivetrainSide == DrivetrainSide.RIGHT) {
            master.setSensorPhase(false)

            master.inverted = true
            slave2.inverted = true

            for (talon in talons) {
                talon.config_kF(Constants.Drivetrain.PID_SLOT_INDEX, Constants.Drivetrain.RIGHT_KF, Constants.TIMEOUT)
            }
        } else {
            slave2.inverted = true

            for (talon in talons) {
                talon.config_kF(Constants.Drivetrain.PID_SLOT_INDEX, Constants.Drivetrain.LEFT_KF, Constants.TIMEOUT)
            }
        }

        for (talon in talons) {
            talon.selectProfileSlot(Constants.Drivetrain.PROFILE_SLOT_INDEX, Constants.Drivetrain.PID_SLOT_INDEX)

            talon.configVoltageCompSaturation(Constants.Drivetrain.VOLTAGE_COMPENSATION, Constants.TIMEOUT)
            talon.configVoltageMeasurementFilter(Constants.Drivetrain.FILTER_WINDOW_SAMPLES, Constants.TIMEOUT)
            talon.enableVoltageCompensation(true)
        }

        master.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10, Constants.TIMEOUT)
        master.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.TIMEOUT)
    }

    fun setNeutralMode(neutralMode: NeutralMode) {
        for (motor in talons) {
            motor.setNeutralMode(neutralMode)
        }
    }

    fun set(mode: ControlMode, value: Double, demand: Double = 0.0) {
        master.set(mode, value, DemandType.ArbitraryFeedForward, demand)
    }

    fun stop(){
        set(ControlMode.PercentOutput, 0.0)
    }

    fun resetEncoder() {
        master.setSelectedSensorPosition(0, Constants.Drivetrain.PID_SLOT_INDEX, Constants.TIMEOUT)
    }

    fun reset() {
        stop()
        resetEncoder()
    }

    fun setRamping(ramping: Boolean) {
        val rampRate = if(ramping) Constants.Drivetrain.RAMP_RATE else 0.0

        for (motor in talons) {
            motor.configOpenloopRamp(rampRate, Constants.TIMEOUT)
        }
    }
}