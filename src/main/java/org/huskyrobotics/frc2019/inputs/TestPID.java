/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.huskyrobotics.frc2019.inputs;

/**
 * Add your docs here.
 */
public class TestPID {

	public enum FeedForwardMode {
		LINEAR, SINE, COSINE;
	}

	public enum FeedForwardBehavior {
		NORMAL, ALWAYSPOSITIVE, INVERTED
	}

	double kp, ki, kd, kf, pOutput, iOutput, fOutput, iAccum, dOutput, integralZone,
			maxIntegralAccum, minOutput, maxOutput, setpoint, input, error, output;
	double lastOutput, lastMeasured;
	double tolerence;
	double dt = 0.025d; // Set delta time to 50 hz, this is likely good enough
	FeedForwardMode feedforwardmode;
	FeedForwardBehavior feedforwardbehavior;
	/** The number of input units per quarter sine/cosine wave */
	private double unitsPerQuarterWave;
	private double maxAcceleration;

	/**
	 * Create a basic PI controller, sans the derivative term. When |error| < integralZone, 
	 * the Integral term will be active. If this is no longer true, the interal accum will
	 * be flushed and the controller will effectively be a P controller. Slightly less
	 * shitty version of drivetrain.shitty_p_loop. Use like this:
	 * <code>
	 * TerriblePID mPid = new TerriblePID(1 (kp), 0 (ki), 0 (kd), 0 (kf), -1 (min output), 1  (max output), 0  (integral zone), 1000  (max integral accum), 0 (ramp rate, null, null)
	 * </code>
	 * @param kp gain
	 * @param ki gain
	 * @param kd gain 
	 * @param kf
	 * @param minOutput
	 * @param maxOutput
	 * @param integralZone about which integral will be active
	 * @param maxIntegralAccum same as minimum accum, i term will top/bottom out here
	 * @param maxAcceleration
	 * @param FeedForwardMode
	 * @param FeedForwardBehavior
	 */
	public TestPID(double kp, double ki, double kd, double kf, double minOutput, double maxOutput,
			double integralZone, double maxIntegralAccum, double maxAcceleration, FeedForwardMode feedForwardMode,
			FeedForwardBehavior feedforwardbehavior) {
		this.kp = kp;
		this.ki = ki;
		this.kd = kd;
		this.kf = kf;
		this.minOutput = minOutput;
		this.maxOutput = maxOutput;
		this.integralZone = integralZone;
		this.maxIntegralAccum = maxIntegralAccum;
		this.feedforwardmode = feedForwardMode;
		this.feedforwardbehavior = feedforwardbehavior;
		this.maxAcceleration = maxAcceleration;
		lastMeasured = 0;
	}

	public TestPID(double kp, double maxOutput) {
		this(kp, 0.0, 0.0, 0.0, -maxOutput, maxOutput, 0.0, 0.0, 0.0, null, null);
	}

	public TestPID(double kp, double minOutput, double maxOutput) {
		this(kp, 0.0, 0.0, 0.0, minOutput, maxOutput, 0.0, 0.0, 0.0, null, null);
	}

	/**
	 * Set the setpoint for this instance of the PID loop. Should be preserved.
	 * @param setpoint
	 */
	public void setSetpoint(double demand) {
		setpoint = demand;
	}

	/** Set the kp gain of the controller */
	public void setKpGain(double kpGain) {
		kp = kpGain;
	}

	public void setTolerence(double tolerence_) {
		tolerence = Math.abs(tolerence_);
	}

	public boolean isOnTarget() {
		return Math.abs(setpoint - lastMeasured) < tolerence;
	}

	/**
	 * Set the maximum output of the controller
	 * @param maxOutput
	 */
	public void setMaxOutput(double max) {
		maxOutput = max;
	}

	/**
	 * Returns the current setpoint of the PID loop
	 * @return setpoint
	 */
	public double getSetpoint() {
		return setpoint;
	}

	/**
	 * Get the last calculated error of the PID loop
	 * @return error
	 */
	public double getError() {
		return error;
	}

	/**
	 * Get the last calculated output of the PID loop
	 * @return output
	 */
	public double getOutput() {
		return output;
	}

	/** 
	 * Get the value of the integral accumulator
	 * @return integral accum
	 */
	public double getIntegralAccum() {
		return iAccum;
	}

	/**
	 * Clamps the integral accumulator to the max output/min accum as set up on class construction
	 * @param iAccum
	 * @return iAccum but clamped
	 */
	private double clampIntegral(double i) {
		if (i > maxIntegralAccum) {
			i = maxIntegralAccum;
		} else if (i < (-1 * maxIntegralAccum)) {
			i = -1 * maxIntegralAccum;
		}
		return i;
	}

	/**
	 * Clamps the output to the max output/min output set up on class construction
	 * @param output
	 * @return output but clamped
	 */
	private double clampOutput(double output) {
		if (output > maxOutput) {
			return maxOutput;
		} else if (output < minOutput) {
			return minOutput;
		} else {
			return output;
		}
	}

	/**
	 * Calculates the feed forward term based on set settings
	 */
	private double calculateFeedForward(double input) {
		if (feedforwardmode != null && feedforwardbehavior != null) {

			switch (feedforwardmode) {
			case SINE:
				fOutput = Math.sin(Math.toRadians(input * unitsPerQuarterWave));
				break;
			case COSINE:
				fOutput = Math.cos(Math.toRadians(input * unitsPerQuarterWave));
				break;
			default:
				fOutput = input * kf;
			}
			switch (feedforwardbehavior) {
			case ALWAYSPOSITIVE:
				fOutput = Math.abs(fOutput);
				break;
			case INVERTED:
				fOutput = fOutput * -1;
				break;
			case NORMAL:
				break;
			}
			return fOutput;
		} else { // feedforward mode/behavor are null, return
			return 0;
		}
	}

	private double ramp(double output, double previousOutput) {
		if (output - previousOutput > maxAcceleration) {
			output += maxAcceleration;
		} else if (output - previousOutput < -maxAcceleration) {
			output -= maxAcceleration;
		}
		return output;
	}

	/**
	 * Updates the PIF(sans D) loop taking only the measured param. It will
	 * calculated the p term, incrament the i accum, and clamp it, and return
	 * the clamped output
	 * @param measured
	 * @return output
	 */
	public double update(double measured) {
		error = setpoint - measured;

		/*
		 * P output is just the error times porportional gain
		 */
		pOutput = kp * error;

		if (kd != 0) {
			dOutput = -1 * kd * (measured - lastMeasured);
		}
		lastMeasured = measured;

		/** The iAccum should start at 0, but is incramented by error 
		 * times dt. This is then clamped to the minimum/maximum of 
		 * the i term. This only happens if the integral gain is set.
		 */
		if (ki != 0) {
			iAccum += error * ki * dt; // incrament the I term by error times integral gain times delta time (numerical integration yeet)
			// System.out.println(String.format("Error (%s) ki (%s) dt (%s) iAccum (%s)", error, ki, dt, iAccum));
			iAccum = clampIntegral(iAccum + iOutput); // clamp the term to the min/max   
		}
		// System.out.println("Ki: " + ki + " iAccum: " + iAccum);

		if ((kf != 0) && (feedforwardbehavior != null) && (feedforwardmode != null)) {
			fOutput = calculateFeedForward(measured);
		}

		/*
		 * This will make sure the output of the loop does not exceed the specified min/max.
		 */
		output = clampOutput(pOutput + iAccum + fOutput);

		if (maxAcceleration != 0) {
			output = ramp(output, lastOutput);
		}

		lastOutput = output;

		return output;
	}

	@Override
	public String toString() {
		try {
			return String.format("Terrible PID Instance: Setpoint (%s) last measured (%s) kP (%s) kI (%s) kD (%s) kF (%s)  minOutput (%s) maxOutput (%s) iZone (%s) max integral accum (%s) maxAcceleration (%s) feedForwardMode (%s) feedForwardBehavior (%s) output (%s) last output (%s)", setpoint, lastMeasured, kp, ki, kd, kf, minOutput, maxOutput, integralZone, maxIntegralAccum, maxAcceleration, feedforwardmode.toString(), feedforwardbehavior.toString(), output, lastOutput);
		} catch (Exception e) {
			//TODO: handle exception
			return "";
		}
	}
}
