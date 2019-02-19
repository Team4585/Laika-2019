/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.huskyrobotics.frc2019.commands;

import org.ghrobotics.lib.mathematics.units.Rotation2d;
import org.huskyrobotics.frc2019.Robot;
import org.huskyrobotics.frc2019.inputs.*;
import org.huskyrobotics.frc2019.Constants;

import edu.wpi.first.wpilibj.command.Command;

public class Turn extends Command {
  double starting_angle;
	double target_angle_relative;
	double target_angle;
	double target_angle_absolute;
	boolean isAbsolute = false;
	double output;
	double max_turn_speed;
	double raw_left;
  double raw_right;
  
  public Turn(Rotation2d target_angle) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.m_Drive);
    this.target_angle = target_angle.getDegree();
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    starting_angle = Robot.m_Drive.getGyro();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    raw_left = Encoder.distanceToRaw(output, Constants.drivetrain.left_wheel_effective_diameter, Constants.drivetrain.kPPR);
    raw_right = (-1) * Encoder.distanceToRaw(output, Constants.drivetrain.right_wheel_effective_diameter, Constants.drivetrain.kPPR);
    Robot.m_Drive.setPowers(output, -output);

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
