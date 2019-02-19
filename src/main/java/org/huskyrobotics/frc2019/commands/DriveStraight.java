/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.huskyrobotics.frc2019.commands;



import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.VelocityKt;
import org.huskyrobotics.frc2019.Constants;
import org.huskyrobotics.frc2019.Robot;
import org.huskyrobotics.frc2019.inputs.TestPID;
import org.huskyrobotics.lib.DriveSignal;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.command.Command;


/**
 * Drive straight ahead. The target angle is set based on the current robot angle.
 * This uses a position PID loop to set drivetrain speeds. The left and right speeds
 * are manipulated based on a turn PID loop such that the angle remains constant.
 * Distance is in feet.
 * // TODO determine how the difference in encoder positions if the angle is changed will affect pid
 * 
 * @author Matthew Morley
 */
public class DriveStraight extends Command {
	double distance;
	double actionMaxSpeed;
	double timeout = 15;
	double start_left_distance;
	double start_right_distance = Robot.m_Drive.getRight().getFeet();
	double end_distance;
	double start_gyro_angle;
	double target_gyro_angle;
	double current_angle;
	double forward_speed;
	double turn_speed;
	double angle_error;
	double left_speed_raw, right_speed_raw;

	private TestPID forwardPID = new TestPID(
			10,
			4);

	private TestPID turnPID = new TestPID(
			0.1,
			3);

	/**
	 * DriveStraight drives in a straight line. The target angle is the same angle as the gyro
	 * is initilized with. Timeout is 15 seconds.
	 * @param distance in feet
	 * @param speed maximum speed in feet per second
	 */
	public DriveStraight(double distance, double speed, double timeout) {
		this.distance = distance;
		this.actionMaxSpeed = speed;
		this.target_gyro_angle = Robot.m_Drive.getGyro(); // TODO make sure that the angle is set correctly.
		requires(Robot.m_Drive);
	}

	/**
	 * DriveStraight drives in a straight line. Target angle is the angle at which the action
	 * is init'ed at. Speed is default auto speed, and timeout is 15 seconds.
	 * @param distance in feet
	 */
	public DriveStraight(double distance) {
		this.distance = distance;
		this.actionMaxSpeed = 4;
		this.target_gyro_angle = Robot.m_Drive.getGyro(); // TODO make sure that the angle is set correctly on constructor call.
		requires(Robot.m_Drive);
	}

	/**
	 * DriveStraight drives in a straight line. Target angle is the angle at which the action
	 * is init'ed at. Speed is default auto speed, and timeout is 15 seconds.
	 * @param distance in feet
	 * @param angle absolute angle in degrees from auto init
	 * 
	 * <p> MAKE SURE THAT YOU USE AN ABSOLUTE ANGLE WITH THIS CONSTRUCTOR!
	 */
	public DriveStraight(double distance, double angle) {
		this.distance = distance;
		this.actionMaxSpeed = 4;
		this.target_gyro_angle = angle; // TODO make sure that the angle is set correctly.
		requires(Robot.m_Drive);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		start_left_distance = Robot.m_Drive.getLeft().getFeet();
		start_right_distance = Robot.m_Drive.getRight().getFeet();
		current_angle = Robot.m_Drive.getGyro();
		end_distance = start_left_distance + distance;
		setTimeout(timeout); // set the timeout
		forwardPID.setSetpoint(end_distance);
		turnPID.setSetpoint(target_gyro_angle);
		forwardPID.setMaxOutput(actionMaxSpeed);

	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		forward_speed = forwardPID.update(Robot.m_Drive.getLeft().getFeet());
		turn_speed = turnPID.update(Robot.m_Drive.getGyro());
		SmartDashboard.putNumber("TurnPID output", turnPID.update(Robot.m_Drive.getGyro()));

		// left_speed_raw = EncoderLib.distanceToRaw(forward_speed + turn_speed, RobotConfig.driveTrain.left_wheel_effective_diameter / 12, 
		//   RobotConfig.driveTrain.POSITION_PULSES_PER_ROTATION) / 10;
		// right_speed_raw = EncoderLib.distanceToRaw(forward_speed - turn_speed, RobotConfig.driveTrain.right_wheel_effective_diameter / 12, 
		//   RobotConfig.driveTrain.POSITION_PULSES_PER_ROTATION) / 10;

		Robot.m_Drive.setClosedLoop(DriveSignal.BRAKE);

		// System.out.println("FORWARD PID: Setpoint: " + forwardPID.getSetpoint() + " Measured: " + Robot.drivetrain.getLeft().getFeet() + 
		// " Error: " + forwardPID.getError() + " OUTPUT VELOCITY (ft/s): " + forwardPID.getOutput());
		// System.out.println("TURN PID: Setpoint: " + turnPID.getSetpoint() + " Measured: " + Robot.drivetrain.getGyro() + 
		// " Error: " + turnPID.getError() + " OUTPUT VELOCITY (ft/s): " + turnPID.getOutput());
		System.out.println(String.format("FORWARD PID: setpoint (error) output (iAccum) | %s (%s) %s (%s)",
				forwardPID.getSetpoint(), forwardPID.getError(), forwardPID.getOutput(), forwardPID.getIntegralAccum()));
		System.out.println(String.format("TURN PID: setpoint (error) output (iAccum) | %s (%s) %s (%s)",
				turnPID.getSetpoint(), turnPID.getError(), turnPID.getOutput(), turnPID.getIntegralAccum()));

	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		// if ( ((Math.abs(Robot.drivetrain.getRight().getFeet() - this.distance) < RobotConfig.auto.tolerences.position_tolerence) 
		//   // && (Math.abs(Robot.drivetrain.getLeft().getFeet() - this.distance) < RobotConfig.drive_auto_position_tolerence) 
		//   && (Math.abs(Robot.drivetrain.getLeftVelocity()) < RobotConfig.auto.tolerences.velocity_tolerence) 
		//   && (Math.abs(Robot.drivetrain.getRightVelocity()) < RobotConfig.auto.tolerences.position_tolerence)
		//   && (Math.abs(target_gyro_angle - current_angle) < RobotConfig.auto.tolerences.angle_tolerence ))
		//   || (isTimedOut()) 
		// ){ return true; }
		// else { return false; }
		return false;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {}
}