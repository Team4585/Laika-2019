/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.huskyrobotics.frc2019.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.huskyrobotics.frc2019.subsystems.hatch.HatchIO;;

public class ReleaseHatchTest extends Command {
  public ReleaseHatchTest() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(m_kennedy);
  }
  HatchIO m_kennedy;

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    m_kennedy.init();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    
    m_kennedy.release();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {

    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    m_kennedy.reset();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
