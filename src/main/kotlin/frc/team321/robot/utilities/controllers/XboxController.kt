package frc.team321.robot.utilities.controllers

import edu.wpi.first.wpilibj.GenericHID
import frc.team321.robot.utilities.RobotUtil

@Suppress("MemberVisibilityCanBePrivate", "unused", "PropertyName")
class XboxController(port: Int): BaseController(port) {
    companion object {
        private const val LEFT_X_ID = 0
        private const val LEFT_Y_ID = 1
        private const val LEFT_TRIGGER_ID = 2
        private const val RIGHT_TRIGGER_ID = 3
        private const val RIGHT_X_ID = 4
        private const val RIGHT_Y_ID = 5

        private const val A_ID = 1
        private const val B_ID = 2
        private const val X_ID = 3
        private const val Y_ID = 4
        private const val LEFT_BUMPER_ID = 5
        private const val RIGHT_BUMPER_ID = 6
        private const val SELECT_ID = 7
        private const val START_ID = 8
        private const val LEFT_JOYSTICK_BUTTON_ID = 9
        private const val RIGHT_JOYSTICK_BUTTON_ID = 10
    }

    val A = buttons[A_ID]
    val B = buttons[B_ID]
    val X = buttons[X_ID]
    val Y = buttons[Y_ID]
    val leftBumper = buttons[LEFT_BUMPER_ID]
    val rightBumper = buttons[RIGHT_BUMPER_ID]
    val select = buttons[SELECT_ID]
    val start = buttons[START_ID]
    val leftJoystickButton = buttons[LEFT_JOYSTICK_BUTTON_ID]
    val rightJoystickButton = buttons[RIGHT_JOYSTICK_BUTTON_ID]
    val leftLancerTrigger = LancerTrigger(joystick, LEFT_TRIGGER_ID)
    val rightLancerTrigger = LancerTrigger(joystick, RIGHT_TRIGGER_ID)

    fun getAPressed() = getRawButtonPressed(A_ID)
    fun getAReleased() = getRawButtonReleased(A_ID)
    fun getAState() = getRawButton(A_ID)

    fun getBPressed() = getRawButtonPressed(B_ID)
    fun getBReleased() = getRawButtonReleased(B_ID)
    fun getBState() = getRawButton(B_ID)

    fun getXPressed() = getRawButtonPressed(X_ID)
    fun getXReleased() = getRawButtonReleased(X_ID)
    fun getXState() = getRawButton(X_ID)

    fun getYPressed() = getRawButtonPressed(Y_ID)
    fun getYReleased() = getRawButtonReleased(Y_ID)
    fun getYState() = getRawButton(Y_ID)

    fun getLeftBumperPressed() = getRawButtonPressed(LEFT_BUMPER_ID)
    fun getLeftBumperReleased() = getRawButtonReleased(LEFT_BUMPER_ID)
    fun getLeftBumperState() = getRawButton(LEFT_BUMPER_ID)

    fun getRightBumperPressed() = getRawButtonPressed(RIGHT_BUMPER_ID)
    fun getRightBumperReleased() = getRawButtonReleased(RIGHT_BUMPER_ID)
    fun getRightBumperState() = getRawButton(RIGHT_BUMPER_ID)

    fun getSelectPressed() = getRawButtonPressed(SELECT_ID)
    fun getSelectReleased() = getRawButtonReleased(SELECT_ID)
    fun getSelectState() = getRawButton(SELECT_ID)

    fun getStartPressed() = getRawButtonPressed(START_ID)
    fun getStartReleased() = getRawButtonReleased(START_ID)
    fun getStartState() = getRawButton(START_ID)

    fun getLeftJoystickButtonPressed() = getRawButtonPressed(LEFT_JOYSTICK_BUTTON_ID)
    fun getLeftJoystickButtonReleased() = getRawButtonReleased(LEFT_JOYSTICK_BUTTON_ID)
    fun getLeftJoystickButtonState() = getRawButton(LEFT_JOYSTICK_BUTTON_ID)

    fun getRightJoystickButtonPressed() = getRawButtonPressed(RIGHT_JOYSTICK_BUTTON_ID)
    fun getRightJoystickButtonReleased() = getRawButtonReleased(RIGHT_JOYSTICK_BUTTON_ID)
    fun getRightJoystickButtonState() = getRawButton(RIGHT_JOYSTICK_BUTTON_ID)

    fun getRawX(hand: GenericHID.Hand): Double{
        return if(hand == GenericHID.Hand.kRight){
            joystick.getRawAxis(RIGHT_X_ID)
        }else{
            joystick.getRawAxis(LEFT_X_ID)
        }
    }

    override fun getX(hand: GenericHID.Hand): Double{
        return RobotUtil.applyDeadband(getRawX(hand))
    }

    fun getRawY(hand: GenericHID.Hand): Double{
        return if(hand == GenericHID.Hand.kRight){
            -joystick.getRawAxis(RIGHT_Y_ID)
        }else{
            -joystick.getRawAxis(LEFT_Y_ID)
        }
    }

    override fun getY(hand: GenericHID.Hand): Double{
        return RobotUtil.applyDeadband(getRawY(hand))
    }

    fun getRawTriggerValue(hand: GenericHID.Hand): Double{
        return if(hand == GenericHID.Hand.kRight){
            joystick.getRawAxis(RIGHT_TRIGGER_ID)
        }else{
            joystick.getRawAxis(LEFT_TRIGGER_ID)
        }
    }

    fun getTriggerValue(hand: GenericHID.Hand): Double{
        return RobotUtil.applyDeadband(getRawTriggerValue(hand))
    }

    fun setRumble(rumblePower: Double) {
        this.joystick.setRumble(RumbleType.kRightRumble, rumblePower.coerceIn(0.0..1.0))
        this.joystick.setRumble(RumbleType.kLeftRumble, rumblePower.coerceIn(0.0..1.0))
    }
}
