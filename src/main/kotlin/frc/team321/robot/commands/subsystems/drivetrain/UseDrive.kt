package frc.team321.robot.commands.subsystems.drivetrain

import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.command.Command
import edu.wpi.first.wpilibj.drive.DifferentialDrive
import frc.team321.robot.OI
import frc.team321.robot.subsystems.drivetrain.Drivetrain
import frc.team321.robot.utilities.enums.DriveMode

class UseDrive(private val driveMode: DriveMode) : Command() {
    private var drive: DifferentialDrive

    init{
        requires(Drivetrain)
        drive = DifferentialDrive(Drivetrain.leftTransmission.master, Drivetrain.rightTransmission.master)
    }

    override fun execute() {
        when(driveMode){
            DriveMode.ARCADE -> drive.arcadeDrive(OI.xboxController.getY(GenericHID.Hand.kLeft), OI.xboxController.getX(GenericHID.Hand.kRight))
            DriveMode.TANK -> drive.tankDrive(OI.xboxController.getY(GenericHID.Hand.kLeft), OI.xboxController.getY(GenericHID.Hand.kRight))
            DriveMode.CURVATURE -> drive.curvatureDrive(OI.xboxController.getY(GenericHID.Hand.kLeft), OI.xboxController.getX(GenericHID.Hand.kRight), true)
        }
    }

    override fun isFinished(): Boolean {
        return false
    }
}