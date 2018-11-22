package frc.team321.robot.subsystems.manipulator

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.motorcontrol.NeutralMode
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import edu.wpi.first.wpilibj.command.Subsystem
import frc.team321.robot.Constants
import frc.team321.robot.RobotMap
import frc.team321.robot.commands.subsystems.manipulator.UseLinearSlide

@Suppress("unused", "MemberVisibilityCanBePrivate")
object LinearSlide : Subsystem() {
    val master = TalonSRX(RobotMap.LinearSlide.MASTER)
    private val slave = TalonSRX(RobotMap.LinearSlide.SLAVE)

    private val talons = arrayOf(master, slave)

    val encoderPosition: Int
        get() = master.getSelectedSensorPosition(Constants.LinearSlide.PID_SLOT_INDEX)

    val encoderVelocity: Int
        get() = master.getSelectedSensorVelocity(Constants.LinearSlide.PID_SLOT_INDEX)

    init {
        slave.follow(master)
        master.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.LinearSlide.PID_SLOT_INDEX, Constants.TIMEOUT)

        for (talon in talons) {
            talon.setNeutralMode(NeutralMode.Brake)
            talon.configOpenloopRamp(Constants.LinearSlide.RAMP_RATE, Constants.TIMEOUT)

            talon.selectProfileSlot(Constants.LinearSlide.PROFILE_SLOT_INDEX, Constants.LinearSlide.PID_SLOT_INDEX)

            talon.config_kF(Constants.LinearSlide.PROFILE_SLOT_INDEX, Constants.LinearSlide.kF, Constants.TIMEOUT)
            talon.config_kP(Constants.LinearSlide.PROFILE_SLOT_INDEX, Constants.LinearSlide.kP, Constants.TIMEOUT)

            talon.configMotionCruiseVelocity(Constants.LinearSlide.MAX_VELOCITY, Constants.TIMEOUT)
            talon.configMotionAcceleration(Constants.LinearSlide.MAX_ACCELERATION, Constants.TIMEOUT)

            talon.configAllowableClosedloopError(Constants.LinearSlide.PID_SLOT_INDEX, Constants.LinearSlide.ALLOWABLE_CLOSED_LOOP_ERROR, Constants.TIMEOUT)

            talon.configForwardSoftLimitThreshold(Constants.LinearSlide.FORWARD_SOFT_LIMIT, Constants.TIMEOUT)
            talon.configForwardSoftLimitEnable(true, Constants.TIMEOUT)

            talon.configReverseSoftLimitThreshold(Constants.LinearSlide.REVERSE_SOFT_LIMIT, Constants.TIMEOUT)
            talon.configReverseSoftLimitEnable(true, Constants.TIMEOUT)

            talon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.TIMEOUT)
            talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.TIMEOUT)

            talon.configVoltageCompSaturation(Constants.LinearSlide.VOLTAGE_COMPENSATION, Constants.TIMEOUT)
            talon.configVoltageMeasurementFilter(Constants.LinearSlide.FILTER_WINDOW_SAMPLE, Constants.TIMEOUT)
            talon.enableVoltageCompensation(true)

            talon.configNominalOutputForward(Constants.LinearSlide.kGravity, Constants.TIMEOUT)
        }
    }

    fun set(controlMode: ControlMode, value: Double) {
        master.set(controlMode, value)
    }

    fun setRamping(ramping: Boolean) {
        val rampRate = if (ramping) Constants.LinearSlide.RAMP_RATE else 0.0

        for (talon in talons) {
            talon.configOpenloopRamp(rampRate, Constants.TIMEOUT)
        }
    }

    override fun initDefaultCommand() {
        defaultCommand = UseLinearSlide()
    }
}