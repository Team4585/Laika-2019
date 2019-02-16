package org.huskyrobotics.frc2019.subsystems.drive.FalconLibStuff;

import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.team254.lib.physics.DCMotorTransmission;
import com.team254.lib.physics.DifferentialDrive;

import org.ghrobotics.lib.localization.Localization;
import org.ghrobotics.lib.localization.TankEncoderLocalization;
import org.ghrobotics.lib.mathematics.twodim.control.FeedForwardTracker;
import org.ghrobotics.lib.mathematics.twodim.control.PurePursuitTracker;
import org.ghrobotics.lib.mathematics.twodim.control.RamseteTracker;
import org.ghrobotics.lib.mathematics.twodim.control.TrajectoryTracker;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory;
import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.Rotation2dKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.Velocity;
import org.ghrobotics.lib.subsystems.drive.DifferentialTrackerDriveBase;
import org.ghrobotics.lib.wrappers.ctre.FalconSRX;
import org.huskyrobotics.frc2019.RobotMap;
import org.huskyrobotics.frc2019.Constants;
import org.huskyrobotics.frc2019.ConstantsAuto;
import org.huskyrobotics.frc2019.subsystems.drive.FalconLibStuff.FalconGearbox;
import org.huskyrobotics.lib.Util;
import org.huskyrobotics.lib.DriveSignal;
import org.huskyrobotics.frc2019.FalconAuto.*;
import org.huskyrobotics.frc2019.inputs.Gyro;

//import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Subsystem;


import kotlin.ranges.RangesKt;

public class FalconDrive extends Subsystem implements DifferentialTrackerDriveBase{
    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new UseDrive());
    }
    
    private static FalconDrive m_instance;


    public List<Double> lastCommandedVoltages;
    public List<Double> lastVelocity = Arrays.asList(0d, 0d);

    public Gyro m_gyro;
    double m_gyroZero;

    private Localization localization;

    public Localization getLocalization() {
        return localization;
    }
/*
    private static DoubleSolenoid shifterDoubleSolenoid = new DoubleSolenoid(9, 0, 1);
    private static int high = 0;
    private static int low = 1;
*/
    /** 
     * Shifts the gearbox
     * 
     * @param HighorLow The gear to shift to
     */
    /*private static void shift(int HighorLow){
        if(HighorLow == high){
        shifterDoubleSolenoid.set(DoubleSolenoid.Value.kForward);
        }else if(HighorLow == low){
            shifterDoubleSolenoid.set(DoubleSolenoid.Value.kReverse);
        }else{
            System.out.println(HighorLow + "is not a valid shifting choice");
        }
    }
    public static enum Gear {
        Low, High;
    }
    Gear gear;
    */

    private static double kQuickStopThreshold = 0.0; 
    private static double kQuickStopAlpha = 0.0;
    private double quickStopAccumulator = 0.0;

    private RamseteTracker ramseteTracker;
    private FeedForwardTracker feedForwardTracker;
    private PurePursuitTracker purePursuitTracker;



    Notifier localizationNotifier;

    public static enum TrajectoryTrackerMode{
        Ramsete, PurePursuit, FeedForward, PID
    }

    
    /*private TrajectoryTrackerMode trackerMode = TrajectoryTrackerMode.Ramsete;
    public void setTrackerMode(TrajectoryTrackerMode mode) {
        trackerMode = mode;
    }*/

    private DCMotorTransmission m_Transmission;
    private DifferentialDrive m_differentialDrive;
    private FalconGearbox leftTransmission, rightTransmission;

    private TrajectoryTrackerMode kDefaulTrajectoryTrackerMode = TrajectoryTrackerMode.Ramsete;

    /* FalconDrive Instance */

    public synchronized static FalconDrive getInstance() {
        if (m_instance == null) m_instance = new FalconDrive();
        return m_instance;
      }
    private FalconDrive(){
        leftTransmission = new FalconGearbox(RobotMap.kLeftMaster, 
            RobotMap.kLeftSlave, 
            FalconGearbox.EncoderMode.QuadEncoder, 
            FalconGearbox.TransmissionSide.Left, 
            false
        );
        rightTransmission = new FalconGearbox(RobotMap.kRightMaster, 
            RobotMap.kRightSlave, 
            FalconGearbox.EncoderMode.QuadEncoder, 
            FalconGearbox.TransmissionSide.Right, 
            true
        );

        getLeft().getMaster().configClosedloopRamp(0.4, 10);
        getRight().getMaster().configClosedloopRamp(0.4, 10);

        localization = new TankEncoderLocalization(
            () -> Rotation2dKt.getDegree(getGyro(true)),
            () -> getLeft().getDistance(),
            () -> getRight().getDistance()
        );
        localization.reset( new Pose2d() );

        m_Transmission = new DCMotorTransmission(
            1 / ConstantsAuto.kVDrive,
            ConstantsAuto.kWheelRadius * ConstantsAuto.kWheelRadius * ConstantsAuto.kRobotMass / (2.0 * ConstantsAuto.kADrive),
            ConstantsAuto.kStaticFrictionVoltage
        );

        m_differentialDrive = new DifferentialDrive(
            ConstantsAuto.kRobotMass,
            ConstantsAuto.kRobotMomentOfInertia,
            ConstantsAuto.kRobotAngularDrag,
            ConstantsAuto.kWheelRadius,
            ConstantsAuto.kTrackWidth / 2.0,
            m_Transmission,
            m_Transmission
        );

        ramseteTracker = new RamseteTracker(ConstantsAuto.kDriveBeta, ConstantsAuto.kDriveZeta);
        purePursuitTracker = new PurePursuitTracker(ConstantsAuto.kLat, ConstantsAuto.kLookaheadTime, ConstantsAuto.kMinLookaheadDistance);
        feedForwardTracker = new FeedForwardTracker();

    }
      /* Utility related Methods */

      public DifferentialDrive getDifferentialDrive() {
        return m_differentialDrive;
      }
    
      public DCMotorTransmission getTransmissionModel() {
        return m_Transmission;
      }
          
      public FalconGearbox getLeft() {
        return leftTransmission;
      }
    
      public FalconGearbox getRight() {
        return rightTransmission;
      }

      public FalconSRX<Length> getLeftMotor() {
        return getLeft().getMaster();
      }
    
      public FalconSRX<Length> getRightMotor() {
        return getRight().getMaster();
      }

      public RamseteTracker getRamseteTracker() {
        return ramseteTracker;
      }
    
      public TrajectoryTracker getTrajectoryTracker() {
        return getTrajectoryTracker(kDefaulTrajectoryTrackerMode);
      }
    
      public TrajectoryTracker getTrajectoryTracker(TrajectoryTrackerMode mode) {
        switch (mode) {
          case Ramsete:
            return ramseteTracker;
          case FeedForward:
            return feedForwardTracker;
          case PurePursuit:
            return purePursuitTracker;
          default:
            return ramseteTracker;
        }
      }
          
      /**
       * @return The robot's position in a Pose2d Object using the Localization method
       */
      public Pose2d getRobotPosition(){
        return getLocalization().getRobotPosition();
      }
      /**
       * Sets the robot's position in a Pose2d Object using the Localization method
       * @param pose2d 3 parameters (x, y, theta), using x and y as length objects, and theta as a rotation2d object
       */
      public void setRobotPosition(Pose2d pose2d){
        getLocalization().reset(pose2d);
      }
    
      /* Practical Methods */
      
      /**
       * Initializes the Drivetrain by setting high gear and zeroing the encoders
       */
      public void init() {
        zeroEncoders();
        //setHighGear();
      }
    /**
     * Gets gyro heading
     * @return gyro heading
     */
      public double getGyro() {
        return m_gyro.getHeading();
      }
    /**
     * Gets the Gyro value of an element of the drivebase
     * @param inverted Whether or not the location required is inverted, which multiplies the getGyro() method by -1
     * @return New angle accounting for if its inverted 
     */
      public double getGyro(boolean inverted) {
        double kgyroang;
        if(inverted) { kgyroang = getGyro() * (-1); } else { kgyroang = getGyro(); }
        //System.out.println("Gyroangle: " + kgyroang);
        return kgyroang;
      }
    
    /**
     * Sets the encoder output to zero, useful for when a method needs to use clean encoder outputs
     */
      public void zeroEncoders() {
        leftTransmission.zeroEncoder();
        rightTransmission.zeroEncoder();
      }
    /**
     * Zeroes the gyro heading. This is done by setting the gyro heading equal to itself so it creates a relative zero value.
     */
      public void zeroGyro() {
        m_gyro.resetAngle();
	  }
    /**
     * Stops the Drivebase
     */
      public void stop() {
        getLeft().stop();
        getRight().stop();
      }

      /*Shifting Methods*/

      /**
       * Sets the gear position of the drivebase gearboxes
       * @param gear Low or High gear parameter
       */
      /*public void setGear(Gear gear) {
        switch (gear) {
          case High:
            setHighGear();
            break;
          case Low:
            setLowGear();
            break;
        }
      }
    */

      /*public void setHighGear() {
        leftTransmission.setClosedLoopGains(
                Constants.drivetrain.kHGleftKp,
                Constants.drivetrain.kHGleftKi,
                Constants.drivetrain.kHGleftKd,
                Constants.drivetrain.kHGleftKf,
                Constants.drivetrain.kHGleftIzone,
                Constants.drivetrain.kHGleftIntMax
        );
        rightTransmission.setClosedLoopGains(
                Constants.drivetrain.kHGrightKp,
                Constants.drivetrain.kHGrightKi,
                Constants.drivetrain.kHGrightKd,
                Constants.drivetrain.kHGrightKf,
                Constants.drivetrain.kHGrightIzone,
                Constants.drivetrain.kHGrightIntMax
        );
        // Trigger solenoids
        shift(high);
        gear = Gear.High;
      }
    
      public void setLowGear() {
        leftTransmission.setClosedLoopGains(
            Constants.drivetrain.kLGleftKp,
            Constants.drivetrain.kLGleftKi,
            Constants.drivetrain.kLGleftKd,
            Constants.drivetrain.kLGleftKf,
            Constants.drivetrain.kLGleftIzone,
            Constants.drivetrain.kLGleftIntMax
        );
        rightTransmission.setClosedLoopGains(
            Constants.drivetrain.kLGrightKp,
            Constants.drivetrain.kLGrightKi,
            Constants.drivetrain.kLGrightKd,
            Constants.drivetrain.kLGrightKf,
            Constants.drivetrain.kLGrightIzone,
            Constants.drivetrain.kLGrightIntMax
        );
        // Trigger solenoids
        shift(low);
        gear = Gear.Low;
      }
      /**
       * Sets the Master Talon control mode
       * @param mode The control mode of the drivebase (Brake or Coast)
       */

    /* Drivetrain Motion */

      public void setNeutralMode(NeutralMode mode) {
        getLeft().getMaster().setNeutralMode(mode);
        getRight().getMaster().setNeutralMode(mode);
      }
      public void setVoltages(double left_voltage, double right_voltage) {
        getLeft().getMaster().set(ControlMode.PercentOutput, left_voltage / 12);
        getRight().getMaster().set(ControlMode.PercentOutput, right_voltage / 12);
      }
    
      public void setClosedLoop(DriveSignal signal) {
        setClosedLoop(signal.getLeft(), signal.getRight(), signal.getLeftPercent(), signal.getRightPercent(), signal.getBrakeMode());
      }
    
    /**
     * Set the drivetrain talons to closed loop velocity mode, given a Velocity<Length>
     * object to represent a unit-signed speed for the left and right spides.
     * @param left velocity in a Velocity<Length> Falconlib object
     * @param right velocity in a Velocity<Length> Falconlib object
     */
      public void setClosedLoop( Velocity<Length> left, Velocity<Length> right, double leftPercent, double rightPercent, boolean brakeMode) {
        setNeutralMode((brakeMode) ? NeutralMode.Brake : NeutralMode.Coast);
        getLeft().getMaster().set(ControlMode.Velocity, left, DemandType.ArbitraryFeedForward, leftPercent);
        getRight().getMaster().set(ControlMode.Velocity, right, DemandType.ArbitraryFeedForward, rightPercent);
      }
    
      public void setClosedLoop(Velocity<Length> left, Velocity<Length> right) {
        setClosedLoop(left, right, 0, 0, true);
      }
    
      /**
       * setPowers is an even more lazy verssion of set speeds. This will literally set the
       * throttle of the left and right talons (from -1 to 1 ofc, like normal)
       * 
       * @param left_power
       * @param right_power
       */
      public void setPowers(double left_power, double right_power) {
        leftTransmission.getMaster().set(ControlMode.PercentOutput, left_power);
        rightTransmission.getMaster().set(ControlMode.PercentOutput, right_power);
      }
      /**
       * Mimics the WPI Curvature Drive using Falcon Library
       * 
       * @param linearPercent Forward Direction (converts Percent to control loop speed)
       * @param curvaturePercent Amount of Curvature (Converts Percent to control loop speed)
       * @param isQuickTurn If set, overrides constant-curvature turning for turn-in-place maneuvers.
       */
      public void curvatureDrive(double linearPercent, double curvaturePercent, boolean isQuickTurn) {
          double angularPower;
          boolean overPower;
          linearPercent = Util.limit(linearPercent,1);
          linearPercent = Util.deadband(linearPercent, 0.1);
      
          curvaturePercent = Util.deadband(curvaturePercent, 0.1);
    
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
      /**
       * Creates a tank drive object
       * @param leftPercent Percent output of the left side of the drivebase
       * @param rightPercent Percent output of the right side of the drivebase
       */
      public void tankDrive(double leftPercent, double rightPercent){
        lastCommandedVoltages = Arrays.asList(leftPercent * 12, rightPercent * 12);
        getLeft().getMaster().set(ControlMode.PercentOutput, leftPercent);
        getRight().getMaster().set(ControlMode.PercentOutput, rightPercent);
      }


      /* Path Finding related methods */

      /**
       * 
       * @param trajectory A trajectory object created in the Trajectories.java file using FalconDashboard
       * @return A followed trajectory using Ramsete nonlinear control
       */
      public TrajectoryTrackerCommand followTrajectory(TimedTrajectory<Pose2dWithCurvature> trajectory){
        return followTrajectory(trajectory, false);
      }
  
    /**
     * Follows a path
     * @param trajectory A trajectory object created in the Trajectories.java file using FalconDashboard
     * @param reset Whether or not the trajectory is reset
     * @return A followed trajectory using Ramsete nonlinear control
     */
      public TrajectoryTrackerCommand followTrajectory(TimedTrajectory<Pose2dWithCurvature> trajectory, boolean reset){
        return new TrajectoryTrackerCommand(this, () -> trajectory, reset);
      }
  
    /**
     * Follows a path
     * @param trajectory A trajectory object created in the Trajectories.java file using FalconDashboard
     * @param mode Method in which the robot controls its trajectory following
     * @param reset Whether or not the trajectory is reset
     * @return A followed trajectory using the mode control method
     */
      public TrajectoryTrackerCommand followTrajectory(TimedTrajectory<Pose2dWithCurvature> trajectory, TrajectoryTrackerMode mode, boolean reset){
        kDefaulTrajectoryTrackerMode = mode;
        return new TrajectoryTrackerCommand(this, getTrajectoryTracker(mode), () -> trajectory, reset);
      }
    /**
     * I've put too much time into this team, just fucking end me
     */
      public void death(){
        if(Constants.kWillToLive <= 0){
          //Sets left and right sides of the drivetrain to 100% (presumably into my (Copinga's) shins)
          tankDrive(1, 1);
        }
      }
}