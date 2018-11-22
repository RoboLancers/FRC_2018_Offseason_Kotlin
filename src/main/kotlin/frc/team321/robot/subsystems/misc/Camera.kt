package frc.team321.robot.subsystems.misc

import edu.wpi.first.wpilibj.CameraServer

object Camera{
    fun start(){
        CameraServer.getInstance().startAutomaticCapture()
    }
}