/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.huskyrobotics.frc2019.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;
import org.huskyrobotics.frc2019.subsystems.drive.*;
import org.huskyrobotics.frc2019.subsystems.drive.Drivetrain;
import org.huskyrobotics.frc2019.subsystems.drive.FalconLibStuff.FalconDrive;
import org.huskyrobotics.frc2019.Robot;
import org.huskyrobotics.frc2019.OI;

public class UseDrive extends Command {
  public UseDrive(OI oi) {
    requires(Robot.m_Drive);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    m_OI = oi;
  }
  OI m_OI;

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.m_Drive.init();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.m_Drive.curvatureDrive(m_OI.getRobotForward(), m_OI.getRobotTwist(), true);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_Drive.curvatureDrive(0, 0, true);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
