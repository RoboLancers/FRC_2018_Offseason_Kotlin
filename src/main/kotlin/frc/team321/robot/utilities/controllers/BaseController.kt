package frc.team321.robot.utilities.controllers

import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.buttons.Button
import edu.wpi.first.wpilibj.buttons.JoystickButton

abstract class BaseController(port: Int): GenericHID(port) {
    val joystick: Joystick = Joystick(port)
    val buttons: List<JoystickButton> by lazy {
        buttons.indices.map { JoystickButton(joystick, it) }
    }
}