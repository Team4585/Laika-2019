/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.huskyrobotics.frc2019.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.*;


import org.huskyrobotics.frc2019.subsystems.drive.*;
import org.huskyrobotics.frc2019.subsystems.drive.Drivetrain;
import org.huskyrobotics.frc2019.subsystems.drive.FalconLibStuff.FalconDrive;
import org.huskyrobotics.lib.DriveSignal;
import org.huskyrobotics.frc2019.Robot;
import org.huskyrobotics.frc2019.OI;

public class UseDrive extends Command {
  public UseDrive() {
    requires(m_Drive);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }
  FalconDrive m_Drive;
  DriveSignal m_Signal;
  OI m_OI = new OI(0,1);
  private ShuffleboardTab tab = Shuffleboard.getTab("Drive status");

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    m_Drive.init();

    m_Drive.setClosedLoop(DriveSignal.BRAKE);

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    //SmartDashboard.putNumber("Gyro", m_Drive.getGyro(true));
    Shuffleboard.getTab("Drive Status")
      .addPersistent("Gyro", m_Drive.getGyro(true));
    //SmartDashboard.putString("Drive Signal", m_Signal.toString());
    Shuffleboard.getTab("Drive Status")
      .addPersistent("Drive Signal", m_Signal.toString());

    m_Drive.curvatureDrive(m_OI.GetRobotForward(), m_OI.GetRobotTwist(), true);
    
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    m_Drive.curvatureDrive(0, 0, true);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
