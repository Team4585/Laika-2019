/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.huskyrobotics.frc2019.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.huskyrobotics.frc2019.subsystems.superstructure.PivotArm;

public class MoveArmToTop extends Command {
  public MoveArmToTop() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(m_arm);
  }
  PivotArm m_arm;

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    m_arm.setTarget(90);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
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
