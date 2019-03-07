/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.huskyrobotics.frc2019.commands;

import org.huskyrobotics.frc2019.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class RunIntake extends Command {
	double speed, duration;
	/**
	 * Run the intake for a set amount of time
	 * @param demand
	 * @param duration
	 */
	public RunIntake(double speed, double duration) {
		requires(Robot.m_Cargo);
		this.speed = speed;
		this.duration = duration;
		setTimeout(duration);
	}

	@Override
	protected void initialize() {
		Robot.m_Cargo.intake(speed);
	}

	@Override
	protected void execute() {}

	@Override
	protected boolean isFinished() {
		return this.isTimedOut();
	}

	@Override
	protected void end() {
		Robot.m_Cargo.intake(0d);
		Robot.m_Cargo.intake(0d);
	}

	@Override
	protected void interrupted() {}
}
