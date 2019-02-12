package org.huskyrobotics.frc2019.autonomous;
import java.io.File;

import org.huskyrobotics.frc2019.Constants;
import org.huskyrobotics.frc2019.Robot;
import org.huskyrobotics.frc2019.subsystems.drive.FalconLibStuff.*;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.DistanceFollower;
import jaci.pathfinder.modifiers.TankModifier;

public class DriveTrajectoryPathfinder extends Command {
  private Trajectory mLeftTrajectory;

  private Trajectory mRightTrajectory;

  private Trajectory mSourceTrajectory;

  private DistanceFollower mLeftFollower;
  
  private DistanceFollower mRightFollower;

  private double mLeftOutput;
  private double mRightOutput;
  private double mTurn;
  private double mAngularError;

  private TankModifier mModifier;

  /** The start distances of the drivetrain on robot init */
  private double mLeftStartDistance;
  private double mRightStartDistance;

  /** Gyro variables for turn P */
  private double mDesiredHeading;

  double mLeftKp;

  double mLeftKi;

  double mLeftKd;

  double mLeftKv;

  double mLeftKa;

  double mRightKp;

  double mRightKi;

  double mRightKd;

  double mRightKv;

  double mRightKa;




  public DriveTrajectoryPathfinder(Trajectory mTraj) {
    mSourceTrajectory = mTraj;

    mModifier = new TankModifier(mSourceTrajectory);

    mModifier.modify(Constants.drivetrain.wheel_base);    
    mLeftTrajectory = mModifier.getLeftTrajectory();
    mRightTrajectory = mModifier.getRightTrajectory();
    
    requires(Robot.m_Drive);
  }

  public DriveTrajectoryPathfinder(String mFile) {
    requires(Robot.m_Drive);
    
    // So I flipped this because Pathweaver seems to be exporting the left and right
    // flipped for some reason. so fix dis. k thx
    File traj = new File("/home/lvuser/deploy/paths/meme.pf1.csv");
    mSourceTrajectory = Pathfinder.readFromCSV(traj);
    // File leftTraj = new File("/home/lvuser/deploy/paths/test.left.pf1.csv");
    // mLeftTrajectory = Pathfinder.readFromCSV(leftTraj);
    // File rightTraj = new File("/home/lvuser/deploy/paths/test.right.pf1.csv");
    // mRightTrajectory = Pathfinder.readFromCSV(rightTraj);
    File leftTraj = new File("/home/lvuser/deploy/paths/meme.left.pf1.csv");
    File rightTraj = new File("/home/lvuser/deploy/paths/meme.right.pf1.csv");
    mRightTrajectory = Pathfinder.readFromCSV(leftTraj);
    mLeftTrajectory = Pathfinder.readFromCSV(rightTraj);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

    // if(RobotConfig.auto.auto_gear == Gear.LOW){
    //   mLeftKp = RobotConfig.driveTrain.left_talons.velocity_kp_low;
    //   mLeftKi = 0;//RobotConfig.driveTrain.left_talons.velocity_ki_low;
    //   mLeftKd = RobotConfig.driveTrain.left_talons.velocity_kd_low;
    //   mLeftKv = RobotConfig.driveTrain.left_talons.velocity_kv_low;
    //   mLeftKa = 0;//RobotConfig.driveTrain.left_talons.velocity_ki_low;

    //   mRightKp = RobotConfig.driveTrain.left_talons.velocity_kp_low;
    //   mRightKi = 0;//RobotConfig.driveTrain.left_talons.velocity_ki_low;
    //   mRightKd = RobotConfig.driveTrain.left_talons.velocity_kd_low;
    //   mRightKv = RobotConfig.driveTrain.left_talons.velocity_kv_low;
    //   mRightKa = 0;//RobotConfig.driveTrain.left_talons.velocity_ka_low;
    // }

    // // Modify the variables if the gear is high, yes this is a bit of a hack but 
    // else {
    mLeftKp = 1;
    mLeftKi = 0; 
    mLeftKd = 0; 
    mLeftKv = 1.2; 
    mLeftKa = 0; 

    mRightKp = mLeftKp;
    mRightKi = 0; 
    mRightKd = 0;
    mRightKv = mLeftKv;
    mRightKa = 0;
    // }

    mLeftStartDistance = Robot.m_Drive.getLeft().getFeet();
    mRightStartDistance = Robot.m_Drive.getRight().getFeet();

    mLeftFollower = new DistanceFollower(mLeftTrajectory);
    mRightFollower = new DistanceFollower(mRightTrajectory);
    mLeftFollower.configurePIDVA(mLeftKp, mLeftKi, mLeftKd, mLeftKv, mLeftKa);
    mRightFollower.configurePIDVA(mRightKp, mRightKi, mRightKd, mRightKv, mRightKa);

    System.out.println("Pathfinder auto init-ed!");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    try {
      mLeftOutput = mLeftFollower.calculate(Robot.m_Drive.getLeft().getFeet() - mLeftStartDistance) + Constants.drivetrain.kleftstatickv;
      mRightOutput = mRightFollower.calculate(Robot.m_Drive.getRight().getFeet() - mRightStartDistance) + Constants.drivetrain.krightstatickv;
    } catch (ArrayIndexOutOfBoundsException e) {
      mLeftOutput = 0;
      mRightOutput = 0;
    }
    
    mDesiredHeading = Pathfinder.r2d(mLeftFollower.getHeading());
    mAngularError = Pathfinder.boundHalfDegrees(mDesiredHeading - Robot.m_Drive.getGyro());
        
    // TODO make sure that the sign is the correct direction, it should be!
    mTurn = mAngularError * Constants.pathfinder.gyro_correct_kp;
    
    Robot.m_Drive.setVoltages(mLeftOutput + mTurn, mRightOutput - mTurn);

    SmartDashboard.putString("Left target pathfinder data: ", 
      String.format("Velocity (position) heading (current): %s (%s) %s (%s)", 
      mLeftFollower.getSegment().velocity, 
      mLeftFollower.getSegment().position, 
      mLeftFollower.getSegment().heading,
      Robot.m_Drive.getLeft().getFeet()));
    SmartDashboard.putString("Right target pathfinder data: ", 
      String.format("Velocity (position) heading (current): %s (%s) %s (%s)", 
      mRightFollower.getSegment().velocity, 
      mRightFollower.getSegment().position, 
      mRightFollower.getSegment().heading,
      Robot.m_Drive.getRight().getFeet()));

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return mRightFollower.isFinished() && mLeftFollower.isFinished();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    // Robot.drive.setPowerZero();
    Robot.m_Drive.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}