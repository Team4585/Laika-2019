/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.huskyrobotics.frc2019.commands;

import edu.wpi.first.wpilibj.command.Command;

import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory;
import org.huskyrobotics.frc2019.FalconAuto.*;
import org.huskyrobotics.frc2019.subsystems.drive.FalconLibStuff.*;
import org.huskyrobotics.frc2019.subsystems.drive.FalconLibStuff.FalconDrive.TrajectoryTrackerMode;

public class AutoMove extends Command {
  public FalconDrive m_Drive = FalconDrive.getInstance();
  TimedTrajectory<Pose2dWithCurvature> trajectory = Trajectories.generatedTrajectories.get("Hatch");
  public AutoMove() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(m_Drive);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Trajectories.generateAllTrajectories();
    m_Drive.zeroEncoders();


  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    m_Drive.followTrajectory(trajectory, TrajectoryTrackerMode.Ramsete, true);
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
