package org.huskyrobotics.lib.subsystems;

import org.huskyrobotics.lib.Autonomous.TrajectoryTrackerCommand;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import kotlin.ranges.RangesKt;
import org.ghrobotics.lib.localization.Localization;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory;
import org.ghrobotics.lib.subsystems.drive.DifferentialTrackerDriveBase;

public abstract class TankDriveSubsystem extends Subsystem implements DifferentialTrackerDriveBase {
    private static double kQuickStopThreshold = DifferentialDrive.kDefaultQuickStopThreshold;
    private static double kQuickStopAlpha = DifferentialDrive.kDefaultQuickStopAlpha;
    private double quickStopAccumulator = 0;

    public abstract Localization getLocalization();

    @Override
    public Pose2d getRobotPosition(){
        return getLocalization().getRobotPosition();
    }

    public void setRobotPosition(Pose2d pose2d){
        getLocalization().reset(pose2d);
    }

    @Override
    public void zeroOutputs(){
        tankDrive(0, 0);
    }

    public void tankDrive(double leftPercent, double rightPercent){
        getLeftMotor().setPercentOutput(leftPercent);
        getRightMotor().setPercentOutput(rightPercent);
    }

    public void arcadeDrive(double linearPercent, double rotationPercent){
        double maxInput = Math.signum(Math.max(Math.abs(linearPercent), Math.abs(rotationPercent)));

        double leftMotorOutput, rightMotorOutput;

        if (linearPercent >= 0.0) {
            // First quadrant, else second quadrant
            if (rotationPercent >= 0.0) {
                leftMotorOutput = maxInput;
                rightMotorOutput = linearPercent - rotationPercent;
            } else {
                leftMotorOutput = linearPercent + rotationPercent;
                rightMotorOutput = maxInput;
            }
        } else {
            // Third quadrant, else fourth quadrant
            if (rotationPercent >= 0.0) {
                leftMotorOutput = linearPercent + rotationPercent;
                rightMotorOutput = maxInput;
            } else {
                leftMotorOutput = maxInput;
                rightMotorOutput = linearPercent - rotationPercent;
            }
        }

        tankDrive(leftMotorOutput, rightMotorOutput);
    }

    public void curvatureDrive(double linearPercent, double curvaturePercent, boolean isQuickTurn) {
        double angularPower;
        boolean overPower;

        if (isQuickTurn) {
            if (Math.abs(linearPercent) < kQuickStopThreshold) {
                quickStopAccumulator = (1 - kQuickStopAlpha) * quickStopAccumulator +
                        kQuickStopAlpha * RangesKt.coerceIn(curvaturePercent, -1, 1) * 2.0;
            }

            overPower = true;
            angularPower = curvaturePercent;
        } else {
            overPower = false;
            angularPower = Math.abs(linearPercent) * curvaturePercent - quickStopAccumulator;

            if(quickStopAccumulator > 1){
                quickStopAccumulator -= 1;
            }else if(quickStopAccumulator < -1){
                quickStopAccumulator += 1;
            }else{
                quickStopAccumulator = 0;
            }
        }

        double leftMotorOutput = linearPercent + angularPower;
        double rightMotorOutput = linearPercent - angularPower;

        // If rotation is overpowered, reduce both outputs to within acceptable range
        if (overPower) {
            if(leftMotorOutput > 1.0){
                rightMotorOutput -= leftMotorOutput - 1.0;
                leftMotorOutput = 1.0;
            }else if(rightMotorOutput > 1.0) {
                leftMotorOutput -= rightMotorOutput - 1.0;
                rightMotorOutput = 1.0;
            }else if(leftMotorOutput < -1.0) {
                rightMotorOutput -= leftMotorOutput + 1.0;
                leftMotorOutput = -1.0;
            }else if(rightMotorOutput < -1.0){
                leftMotorOutput -= rightMotorOutput + 1.0;
                rightMotorOutput = -1.0;
            }
        }

        // Normalize the wheel speeds
        double maxMagnitude = Math.max(Math.abs(leftMotorOutput), Math.abs(rightMotorOutput));
        if (maxMagnitude > 1.0) {
            leftMotorOutput /= maxMagnitude;
            rightMotorOutput /= maxMagnitude;
        }

        tankDrive(leftMotorOutput, rightMotorOutput);
    }

    public TrajectoryTrackerCommand followTrajectory(TimedTrajectory<Pose2dWithCurvature> trajectory){
        return followTrajectory(trajectory, false);
    }

    public TrajectoryTrackerCommand followTrajectory(TimedTrajectory<Pose2dWithCurvature> trajectory, boolean reset){
        return new TrajectoryTrackerCommand(this, this, () -> trajectory, reset);
    }
}