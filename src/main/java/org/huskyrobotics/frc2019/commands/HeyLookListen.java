/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.huskyrobotics.frc2019.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.huskyrobotics.frc2019.Robot;
import org.huskyrobotics.frc2019.subsystems.drive.FalconLibStuff.*;
import org.huskyrobotics.frc2019.inputs.Vision;
import org.huskyrobotics.lib.Util;
import org.huskyrobotics.frc2019.Constants;

/**
 * An example command.  You can replace me with your own command.
 */
public class HeyLookListen extends Command {
  double timeout,
  targetSpeed,
  targetPercentOfFrame,
  angleDeltaX,
  angleDeltaY,
  forwardSpeed,
  turnSpeed,
  leftSpeedRaw,
  rightSpeedRaw;
  boolean followRange = false;

  double targetSizeSetpoint = 6;

  boolean noCurrentTarget = false;

  boolean tooClose = false;

  boolean hadTarget = false;
  double lastKnownYaw;
  Vision m_Limelight;
  public HeyLookListen(double speed, double timeout) {
      this.timeout = timeout;
      this.targetSpeed = speed;
      requires(Robot.m_Drive);
  }
  public HeyLookListen(double speed, double targetPercentOfFrame, double timeout) {
    this.timeout = timeout;
    this.targetSpeed = speed;
    this.targetPercentOfFrame = targetPercentOfFrame;
    this.followRange = true;
    requires(Robot.m_Drive);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    setTimeout(timeout);

    angleDeltaX = m_Limelight.getDx();
    targetPercentOfFrame = m_Limelight.getTargetArea();


  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (m_Limelight.getData()[0] != 0) {
      noCurrentTarget = false;
      hadTarget = true;

      double[] data = m_Limelight.getData();
      double limelightData = data[1];
      double sizeData = data[3];

      lastKnownYaw = limelightData;
      // turnSpeed = ;
      turnSpeed = Util.limit( limelightData * (1/10), -0.5, 0.5);

      System.out.println("Turn speed: " + turnSpeed);

      forwardSpeed = 0;

      double distanceRatio = targetSizeSetpoint - sizeData;

      double forwardSpeed = distanceRatio * 0.2;

      if (sizeData > targetSizeSetpoint) { tooClose = true; System.out.println("Too close"); }

      if ( forwardSpeed > 0.5 ) { forwardSpeed = 0.5;}
      if ( forwardSpeed < -0.5 ) { forwardSpeed = -0.5;}

      System.out.println("forward speed: " + forwardSpeed + " Turn speed: " + turnSpeed);

    } else {
      noCurrentTarget = true;
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    double datainYaw = m_Limelight.getData()[1];
    double datainSize = m_Limelight.getData()[3];
    


    return (isTimedOut() || (hadTarget && noCurrentTarget) || (((datainSize > targetSizeSetpoint))  && (datainYaw < 0.5) ));
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
