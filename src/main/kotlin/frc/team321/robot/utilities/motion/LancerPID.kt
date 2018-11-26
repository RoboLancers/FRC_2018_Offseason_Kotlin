package frc.team321.robot.utilities.motion

@Suppress("unused", "MemberVisibilityCanBePrivate")
class LancerPID {
    private var kP = 0.0
    private var kI = 0.0
    private var kD = 0.0
    private var kF = 0.0

    private var maxIOutput = 0.0
    private var maxError = 0.0
    private var errorSum = 0.0

    private var maxOutput = 0.0
    private var minOutput = 0.0

    private var setpoint = 0.0

    private var lastActual = 0.0

    private var firstRun = true
    private var reversed = false

    private var outputRampRate = 0.0
    private var lastOutput = 0.0

    private var outputFilter = 0.0

    private var setpointRange = 0.0

    /**
     * Calculate the output value for the current PID cycle.<br></br>
     * In no-parameter mode, this uses the last sensor value,
     * and last setpoint value. <br></br>
     * Not typically useful, and use of parameter modes is suggested. <br></br>
     * @return calculated output value for driving the system
     */
    val output: Double
        get() = getOutput(lastActual, setpoint)

    //**********************************
    // Constructor functions
    //**********************************

    /**
     * Create a MiniPID class object.
     * See setP, setI, setD methods for more detailed parameters.
     * @param p Proportional gain. Large if large difference between setpoint and target.
     * @param i Integral gain.  Becomes large if setpoint cannot reach target quickly.
     * @param d Derivative gain. Responds quickly to large changes in error. Small values prevents kP and kI terms from causing overshoot.
     */
    constructor(p: Double, i: Double, d: Double) {
        kP = p
        kI = i
        kD = d
        checkSigns()
    }

    /**
     * Create a MiniPID class object.
     * See setP, setI, setD, setF methods for more detailed parameters.
     * @param p Proportional gain. Large if large difference between setpoint and target.
     * @param i Integral gain.  Becomes large if setpoint cannot reach target quickly.
     * @param d Derivative gain. Responds quickly to large changes in error. Small values prevents kP and kI terms from causing overshoot.
     * @param f Feed-forward gain. Open loop "best guess" for the output should be. Only useful if setpoint represents a rate.
     */
    constructor(p: Double, i: Double, d: Double, f: Double) {
        kP = p
        kI = i
        kD = d
        kF = f
        checkSigns()
    }

    //**********************************
    // Configuration functions
    //**********************************
    /**
     * Configure the Proportional gain parameter. <br></br>
     * This responds quickly to changes in setpoint, and provides most of the initial driving force
     * to make corrections. <br></br>
     * Some systems can be used with only a kP gain, and many can be operated with only PI.<br></br>
     * For position based controllers, this is the first parameter to tune, with kI second. <br></br>
     * For rate controlled systems, this is often the second after kF.
     *
     * @param p Proportional gain. Affects output according to **output+=kP*(setpoint-current_value)**
     */
    fun setP(p: Double) {
        kP = p
        checkSigns()
    }

    /**
     * Changes the kI parameter <br></br>
     * This is used for overcoming disturbances, and ensuring that the controller always gets to the control mode.
     * Typically tuned second for "Position" based modes, and third for "Rate" or continuous based modes. <br></br>
     * Affects output through **output+=previous_errors*Igain ;previous_errors+=current_error**
     *
     * @see {@link .setMaxIOutput
     * @param i New gain value for the Integral term
     */
    fun setI(i: Double) {
        if (kI != 0.0) {
            errorSum = errorSum * kI / i
        }
        if (maxIOutput != 0.0) {
            maxError = maxIOutput / i
        }
        kI = i
        checkSigns()
        // Implementation note:
        // This Scales the accumulated error to avoid output errors.
        // As an example doubling the kI term cuts the accumulated error in half, which results in the
        // output change due to the kI term constant during the transition.
    }

    /**
     * Changes the kD parameter <br></br>
     * This has two primary effects:
     * <list>
     *  <li> Adds a "startup kick" and speeds up system response during setpoint changes
     *  <li> Adds "drag" and slows the system when moving toward the target
     * </list>
     * A small kD value can be useful for both improving response times, and preventing overshoot.
     * However, in many systems a large kD value will cause significant instability, particularly
     * for large setpoint changes.
     * <br></br>
     * Affects output through **output += -kD*(current_input_value - last_input_value)**
     *
     * @param d New gain value for the Derivative term
     */
    fun setD(d: Double) {
        kD = d
        checkSigns()
    }

    /**
     * Configure the FeedForward parameter. <br></br>
     * This is excellent for velocity, rate, and other continuous control modes where you can
     * expect a rough output value based solely on the setpoint.<br></br>
     * Should not be used in "position" based control modes.<br></br>
     * Affects output according to **output+=kF*Setpoint**. Note, that a kF-only system is actually open loop.
     *
     * @param f Feed forward gain.
     */
    fun setF(f: Double) {
        kF = f
        checkSigns()
    }

    /**
     * Configure the PID object.
     * See setP, setI, setD methods for more detailed parameters.
     * @param p Proportional gain. Large if large difference between setpoint and target.
     * @param i Integral gain. Becomes large if setpoint cannot reach target quickly.
     * @param d Derivative gain. Responds quickly to large changes in error. Small values prevents kP and kI terms from causing overshoot.
     */
    fun setPID(p: Double, i: Double, d: Double) {
        kP = p
        kD = d
        //Note: the kI term has additional calculations, so we need to use it's
        //specific method for setting it.
        setI(i)
        checkSigns()
    }

    /**
     * Configure the PID object.
     * See setP, setI, setD, setF methods for more detailed parameters.
     * @param p Proportional gain. Large if large difference between setpoint and target.
     * @param i Integral gain.  Becomes large if setpoint cannot reach target quickly.
     * @param d Derivative gain. Responds quickly to large changes in error. Small values prevents kP and kI terms from causing overshoot.
     * @param f Feed-forward gain. Open loop "best guess" for the output should be. Only useful if setpoint represents a rate.
     */
    fun setPID(p: Double, i: Double, d: Double, f: Double) {
        kP = p
        kD = d
        kF = f
        //Note: the kI term has additional calculations, so we need to use it's
        //specific method for setting it.
        setI(i)
        checkSigns()
    }

    /**
     * Set the maximum output value contributed by the kI component of the system
     * This can be used to prevent large windup issues and make tuning simpler
     * @param maximum Units are the same as the expected output value
     */
    fun setMaxIOutput(maximum: Double) {
        // Internally maxError and Izone are similar, but scaled for different purposes.
        // The maxError is generated for simplifying math, since calculations against
        // the max error are far more common than changing the kI term or Izone.
        maxIOutput = maximum
        if (kI != 0.0) {
            maxError = maxIOutput / kI
        }
    }

    /**
     * Specify a maximum output range. <br></br>
     * When one input is specified, output range is configured to
     * **[-output, output]**
     * @param output
     */
    fun setOutputLimits(output: Double) {
        setOutputLimits(-output, output)
    }

    /**
     * Specify a  maximum output.
     * When two inputs specified, output range is configured to
     * **[minimum, maximum]**
     * @param minimum possible output value
     * @param maximum possible output value
     */
    fun setOutputLimits(minimum: Double, maximum: Double) {
        if (maximum < minimum) return
        maxOutput = maximum
        minOutput = minimum

        // Ensure the bounds of the kI term are within the bounds of the allowable output swing
        if (maxIOutput == 0.0 || maxIOutput > maximum - minimum) {
            setMaxIOutput(maximum - minimum)
        }
    }

    /**
     * Set the operating direction of the PID controller
     * @param reversed Set true to reverse PID output
     */
    fun setDirection(reversed: Boolean) {
        this.reversed = reversed
    }

    //**********************************
    // Primary operating functions
    //**********************************

    /**
     * Configure setpoint for the PID calculations<br></br>
     * This represents the target for the PID system's, such as a
     * position, velocity, or angle. <br></br>
     * @param setpoint
     */
    fun setSetpoint(setpoint: Double) {
        this.setpoint = setpoint
    }

    /**
     * Calculate the output value for the current PID cycle.<br></br>
     * @param actual The monitored value, typically as a sensor input.
     * @param setpoint The target value for the system
     * @return calculated output value for driving the system
     */
    fun getOutput(actual: Double, setpoint: Double = this.setpoint): Double {
        this.setpoint = setpoint

        var output: Double
        val pOutput: Double
        var iOutput: Double
        val dOutput: Double = -kD * (actual - lastActual)
        val fOutput: Double = kF * setpoint

        this.setpoint = setpoint

        // Ramp the setpoint used for calculations if user has opted to do so
        if (setpointRange != 0.0) {
            this.setpoint = this.setpoint.coerceIn((actual - setpointRange)..(actual + setpointRange))
        }

        // Do the simple parts of the calculations
        val error = setpoint - actual

        // Calculate kF output. Notice, this depends only on the setpoint, and not the error.

        // Calculate kP term
        pOutput = kP * error

        // If this is our first time running this, we don't actually _have_ a previous input or output.
        // For sensor, sanely assume it was exactly where it is now.
        // For last output, we can assume it's the current time-independent outputs.
        if (firstRun) {
            lastActual = actual
            lastOutput = pOutput + fOutput
            firstRun = false
        }

        // Calculate kD Term
        // Note, this is negative. This actually "slows" the system if it's doing
        // the correct thing, and small values helps prevent output spikes and overshoot
        lastActual = actual

        // The Iterm is more complex. There's several things to factor in to make it easier to deal with.
        // 1. maxIoutput restricts the amount of output contributed by the Iterm.
        // 2. prevent windup by not increasing errorSum if we're already running against our max Ioutput
        // 3. prevent windup by not increasing errorSum if output is output=maxOutput
        iOutput = kI * errorSum
        if (maxIOutput != 0.0) {
            iOutput = iOutput.coerceIn(-maxIOutput..maxIOutput)
        }

        // And, finally, we can just add the terms up
        output = fOutput + pOutput + iOutput + dOutput

        // Figure out what we're doing with the error.
        if (minOutput != maxOutput && !bounded(output, minOutput, maxOutput)) {
            errorSum = error
            // reset the error sum to a sane level
            // Setting to current error ensures a smooth transition when the kP term
            // decreases enough for the kI term to start acting upon the controller
            // From that point the kI term will build up as would be expected
        } else if (outputRampRate != 0.0 && !bounded(output, lastOutput - outputRampRate, lastOutput + outputRampRate)) {
            errorSum = error
        } else if (maxIOutput != 0.0) {
            errorSum = (errorSum + error).coerceIn(-maxError..maxError)
            // In addition to output limiting directly, we also want to prevent kI term
            // buildup, so restrict the error directly
        } else {
            errorSum += error
        }

        // Restrict output to our specified output and ramp limits
        if (outputRampRate != 0.0) {
            output = output.coerceIn((lastOutput - outputRampRate)..(lastOutput + outputRampRate))
        }
        if (minOutput != maxOutput) {
            output = output.coerceIn(minOutput..maxOutput)
        }
        if (outputFilter != 0.0) {
            output = lastOutput * outputFilter + output * (1 - outputFilter)
        }

        // Get a test printline with lots of details about the internal
        // calculations. This can be useful for debugging.
        // System.out.printf("Final output %5.2f [ %5.2f, %5.2f , %5.2f  ], eSum %.2f\n",output,Poutput, Ioutput, Doutput,errorSum );
        // System.out.printf("%5.2f\t%5.2f\t%5.2f\t%5.2f\n",output,Poutput, Ioutput, Doutput );

        lastOutput = output
        return output
    }

    /**
     * Resets the controller. This erases the kI term buildup, and removes
     * kD gain on the next loop.<br></br>
     * This should be used any time the PID is disabled or inactive for extended
     * duration, and the controlled portion of the system may have changed due to
     * external forces.
     */
    fun reset() {
        firstRun = true
        errorSum = 0.0
    }

    /**
     * Set the maximum rate the output can increase per cycle.<br></br>
     * This can prevent sharp jumps in output when changing setpoints or
     * enabling a PID system, which might cause stress on physical or electrical
     * systems.  <br></br>
     * Can be very useful for fast-reacting control loops, such as ones
     * with large kP or kD values and feed-forward systems.
     *
     * @param rate, with units being the same as the output
     */
    fun setOutputRampRate(rate: Double) {
        outputRampRate = rate
    }

    /**
     * Set a limit on how far the setpoint can be from the current position
     * <br></br>Can simplify tuning by helping tuning over a small range applies to a much larger range.
     * <br></br>This limits the reactivity of kP term, and restricts impact of large kD term
     * during large setpoint adjustments. Increases lag and kI term if range is too small.
     * @param range, with units being the same as the expected sensor range.
     */
    fun setSetpointRange(range: Double) {
        setpointRange = range
    }

    /**
     * Set a filter on the output to reduce sharp oscillations. <br></br>
     * 0.1 is likely a sane starting value. Larger values use historical data
     * more heavily, with low values weigh newer data. 0 will disable, filtering, and use
     * only the most recent value. <br></br>
     * Increasing the filter strength will kP and kD oscillations, but force larger kI
     * values and increase kI term overshoot.<br></br>
     * Uses an exponential wieghted rolling sum filter, according to a simple <br></br>
     * <pre>output*(1-strength)*sum(0..n){output*strength^n}</pre> algorithm.
     * @param strength valid between [0..1), meaning [current output only.. historical output only)
     */
    fun setOutputFilter(strength: Double) {
        if (strength == 0.0 || bounded(strength, 0.0, 1.0)) {
            outputFilter = strength
        }
    }

    /**
     * Test if the value is within the min and max, inclusive
     * @param value to test
     * @param min Minimum value of range
     * @param max Maximum value of range
     * @return true if value is within range, false otherwise
     */
    private fun bounded(value: Double, min: Double, max: Double): Boolean {
        return min < value && value < max
    }

    /**
     * To operate correctly, all PID parameters require the same sign
     * This should align with the reversed value
     */
    private fun checkSigns() {
        if (reversed) {  // all values should be below zero
            if (kP > 0) kP *= -1.0
            if (kI > 0) kI *= -1.0
            if (kD > 0) kD *= -1.0
            if (kF > 0) kF *= -1.0
        } else {  // all values should be above zero
            if (kP < 0) kP *= -1.0
            if (kI < 0) kI *= -1.0
            if (kD < 0) kD *= -1.0
            if (kF < 0) kF *= -1.0
        }
    }
}