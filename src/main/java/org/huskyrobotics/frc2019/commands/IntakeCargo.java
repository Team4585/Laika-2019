/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.huskyrobotics.frc2019.commands;

import edu.wpi.first.wpilibj.command.TimedCommand;
import org.huskyrobotics.frc2019.Robot;

/**
 * Add your docs here.
 */
public class IntakeCargo extends TimedCommand {
  /**
   * Add your docs here.
   */
  public IntakeCargo(double timeout) {
    super(timeout);
    requires(Robot.m_Sputnik);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.m_Sputnik.stop();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.m_Sputnik.intake();
  }

  // Called once after timeout
  @Override
  protected void end() {
    Robot.m_Sputnik.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.m_Sputnik.stop();
  }
}
