package frc.team321.robot.subsystems.misc

import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.wpilibj.SerialPort

object Sensors{
    val navX = AHRS(SerialPort.Port.kUSB)

    val angle: Double
        @Synchronized get() = -navX.angle
}