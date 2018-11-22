package frc.team321.robot.utilities.controllers

import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.buttons.Trigger
import edu.wpi.first.wpilibj.command.Command

@Suppress("unused")
class LancerTrigger constructor(private val joystick: Joystick, private val triggerPort: Int, private val tolerance: Double = 0.1) : Trigger() {
    override fun get(): Boolean {
        return joystick.getRawAxis(triggerPort) >= tolerance
    }

    fun whileHeld(command: Command) {
        whileActive(command)
    }

    fun whenPressed(command: Command) {
        whenActive(command)
    }

    fun whenReleased(command: Command){
        whenInactive(command)
    }
}