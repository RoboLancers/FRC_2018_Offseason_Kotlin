package frc.team321.robot.utilities.controllers

import frc.team321.robot.utilities.RobotUtil

@Suppress("unused", "MemberVisibilityCanBePrivate")
class FlightController(port: Int) : BaseController(port) {
    companion object {
        private const val STICK_X_ID = 0
        private const val STICK_Y_ID = 1
        private const val STICK_TURN_ID = 2
        private const val RUDDER_ID = 3
        private const val TRIGGER_ID = 1
        private const val SHOOTER_ID = 2
        private const val BOTTOM_LEFT_ID = 3
        private const val BOTTOM_RIGHT_ID = 4
        private const val TOP_LEFT_ID = 5
        private const val TOP_RIGHT_ID = 6
        private const val FAR_TOP_ID = 7
        private const val INNER_TOP_ID = 8
        private const val FAR_MIDDLE_ID = 9
        private const val INNER_MIDDLE_ID = 10
        private const val FAR_BOTTOM_ID = 11
        private const val INNER_BOTTOM_ID = 12
    }

    val trigger = buttons[TRIGGER_ID]
    val shooter = buttons[SHOOTER_ID]
    val bottomLeft = buttons[BOTTOM_LEFT_ID]
    val bottomRight = buttons[BOTTOM_RIGHT_ID]
    val topLeft = buttons[TOP_LEFT_ID]
    val topRight = buttons[TOP_RIGHT_ID]
    val farTop = buttons[FAR_TOP_ID]
    val innerTop = buttons[INNER_TOP_ID]
    val farMiddle = buttons[FAR_MIDDLE_ID]
    val innerMiddle = buttons[INNER_MIDDLE_ID]
    val farBottom = buttons[FAR_BOTTOM_ID]
    val innerBottom = buttons[INNER_BOTTOM_ID]

    val rawXAxisValue: Double
        get() = joystick.getRawAxis(STICK_X_ID).coerceIn(-1.0..1.0)

    val rawYAxisValue: Double
        get() = -joystick.getRawAxis(STICK_Y_ID).coerceIn(-1.0..1.0)

    val rawRotateAxisValue: Double
        get() = joystick.getRawAxis(STICK_TURN_ID).coerceIn(-1.0..1.0)

    val xAxisValue: Double
        get() = RobotUtil.applyDeadband(rawXAxisValue)

    val yAxisValue: Double
        get() = RobotUtil.applyDeadband(rawYAxisValue)

    val rotateAxisValue: Double
        get() = RobotUtil.applyDeadband(rawRotateAxisValue)

    val rudderAxis: Double
        get() = (joystick.getRawAxis(RUDDER_ID) + 1) / 4 + 0.5

    //Hand argument makes not sense here since there's no left or right axis
    override fun getX(hand: Hand): Double = xAxisValue
    override fun getY(hand: Hand): Double = yAxisValue
}