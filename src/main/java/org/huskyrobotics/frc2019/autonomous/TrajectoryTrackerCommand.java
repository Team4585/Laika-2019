/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.huskyrobotics.frc2019.autonomous;

import java.util.function.Supplier;

import org.ghrobotics.lib.debug.LiveDashboard;
import org.ghrobotics.lib.mathematics.twodim.control.TrajectoryTracker;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedEntry;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TrajectorySamplePoint;
import org.ghrobotics.lib.mathematics.units.TimeUnitsKt;
import org.ghrobotics.lib.subsystems.drive.TrajectoryTrackerOutput;

import edu.wpi.first.wpilibj.command.Command;
import org.huskyrobotics.frc2019.Robot;
import org.huskyrobotics.frc2019.subsystems.drive.FalconLibStuff.*;

// @SuppressWarnings({"WeakerAccess", "unused"})
public class TrajectoryTrackerCommand extends Command {
  private TrajectoryTracker trajectoryTracker;
  private Supplier<TimedTrajectory<Pose2dWithCurvature>> trajectorySource;
  private FalconDrive driveBase;
  private boolean reset;
  private TrajectoryTrackerOutput output;
  // private DifferentialDrive mModel;

  public TrajectoryTrackerCommand(FalconDrive driveBase, Supplier<TimedTrajectory<Pose2dWithCurvature>> trajectorySource){
      this(driveBase, trajectorySource, false);
  }

  public TrajectoryTrackerCommand(FalconDrive driveBase, Supplier<TimedTrajectory<Pose2dWithCurvature>> trajectorySource, boolean reset){
      this(driveBase, Robot.m_Drive.getTrajectoryTracker(), trajectorySource, reset);
  }

  public TrajectoryTrackerCommand(FalconDrive driveBase, TrajectoryTracker trajectoryTracker, Supplier<TimedTrajectory<Pose2dWithCurvature>> trajectorySource, boolean reset){
      requires(driveBase);
      this.driveBase = driveBase;
      this.trajectoryTracker = trajectoryTracker;
      this.trajectorySource = trajectorySource;
      this.reset = reset;
  }

  @Override
  protected void initialize(){
    LiveDashboard.INSTANCE.setFollowingPath(false);

    driveBase.getLeft().getMaster().configClosedloopRamp(0.5);
    driveBase.getRight().getMaster().configClosedloopRamp(0.5);

    if(trajectorySource == null) {
      System.out.println("Sadly the trajectories are not generated");
      Trajectories.generateAllTrajectories();
    }

    System.out.println("get: " + trajectorySource.get().getFirstState().getState().getCurvature().toString());

    trajectoryTracker.reset(this.trajectorySource.get());

    System.out.println("first pose: " + trajectorySource.get().getFirstState().getState().getPose().getTranslation().getX().getFeet() );

    if(reset == true) {
        Robot.m_Drive.getLocalization().reset( trajectorySource.get().getFirstState().getState().getPose() );
    }

    LiveDashboard.INSTANCE.setFollowingPath(true);
  }

  @Override
  protected void execute(){

    long now = System.currentTimeMillis();

    output = trajectoryTracker.nextState(driveBase.getRobotPosition(), TimeUnitsKt.getMillisecond(System.currentTimeMillis()));

    TrajectorySamplePoint<TimedEntry<Pose2dWithCurvature>> referencePoint = trajectoryTracker.getReferencePoint();
    if(referencePoint != null){
      Pose2d referencePose = referencePoint.getState().getState().getPose();

      LiveDashboard.INSTANCE.setPathX(referencePose.getTranslation().getX().getFeet());
      LiveDashboard.INSTANCE.setPathY(referencePose.getTranslation().getY().getFeet());
      LiveDashboard.INSTANCE.setPathHeading(referencePose.getRotation().getRadian());
    }
    // Logger.log("Linear: " + output.getLinearVelocity().getValue() + " Angular: " + output.getAngularVelocity().getValue() );
    driveBase.setOutput(output);

    long elapsed = System.currentTimeMillis() - now;
    System.out.println("Took " + elapsed + "ms");
  }

  @Override
  protected void end(){
      driveBase.stop();
      LiveDashboard.INSTANCE.setFollowingPath(false);
  }

  @Override
  protected boolean isFinished() {
      return trajectoryTracker.isFinished();
  }
}
