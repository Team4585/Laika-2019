/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.huskyrobotics.frc2019.subsystems.drive;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import org.huskyrobotics.lib.subsystems.*;
import org.huskyrobotics.frc2019.Constants;
import org.huskyrobotics.frc2019.RobotMap;
import org.huskyrobotics.frc2019.inputs.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.*;

public class Drivetrain extends Subsystem {
    public void initDefaultCommand() 
	{
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
	}
   //Defining Master Talon SRXs
   private WPI_TalonSRX LeftMaster = new WPI_TalonSRX(RobotMap.kLeftMaster);
   private WPI_TalonSRX RightMaster = new WPI_TalonSRX(RobotMap.kRightMaster);

   //Defining Slave Victor SPXs
   private VictorSPX LeftSlave = new VictorSPX(RobotMap.kLeftSlave);
   private VictorSPX RightSlave = new VictorSPX(RobotMap.kRightMaster);
   
   //Creates a double to keep track of the angle gathered by the Pigeon
   public double supposedAngle;


   //Defining Control from WPI Talon SRXs
   public DifferentialDrive drive = new DifferentialDrive(LeftMaster, RightMaster);

   //Creating an inverted boolean
   Boolean inverted = true;
   public void DTinit(){
       
    // Configure Left Side
        LeftMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, Constants.drivePIDIdx, Constants.kTimeoutMs);
        LeftMaster.setSensorPhase(true);
        LeftMaster.configNominalOutputForward(0, Constants.kTimeoutMs);
        LeftMaster.configNominalOutputReverse(0, Constants.kTimeoutMs);
        LeftMaster.configPeakOutputForward(1, Constants.kTimeoutMs);
        LeftMaster.configPeakOutputReverse(-1,Constants.kTimeoutMs);

    // Configure Right Side 
        RightMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, Constants.drivePIDIdx, Constants.kTimeoutMs);
        RightMaster.setSensorPhase(true);
        RightMaster.configNominalOutputForward(0, Constants.kTimeoutMs);
        RightMaster.configNominalOutputReverse(0, Constants.kTimeoutMs);
        RightMaster.configPeakOutputForward(1, Constants.kTimeoutMs);
        RightMaster.configPeakOutputReverse(-1,Constants.kTimeoutMs);
    
    // Slaving Victors
        LeftSlave.follow(LeftMaster);
        RightSlave.follow(RightMaster);
    
    // Overcomplicating Inverted Methods
        if(inverted == true){
        RightSlave.setInverted(inverted);
        RightMaster.setInverted(inverted);
        LeftSlave.setInverted(!inverted);
        LeftMaster.setInverted(!inverted);
        }else{
        RightSlave.setInverted(!inverted);
        RightMaster.setInverted(!inverted);
        LeftSlave.setInverted(inverted);
        LeftMaster.setInverted(inverted);
        }
    }
    public void customArcadeDrive(double xValue, double yValue, Gyro gyro)
	{
		if(yValue != 0 && Math.abs(xValue) < 0.15) //Checks that the xValue is less than 15%, giving some turning deadband
        {
			setSpeed(yValue, yValue);
	 	}
		else if(yValue == 0 && Math.abs(xValue) < 0.15) //Resets the angle when there's no yValue
		{
			gyro.resetAngle();
			stop();
			supposedAngle = gyro.getYaw();
		}
		else
		{
			gyro.resetAngle();
			curvatureDrive(xValue, yValue);
			supposedAngle = gyro.getYaw();
		}
	}
    
    //Sets the percent output of each motor
    public void setPercentOutput(double lOutput, double rOutput)
	{
	    RightMaster.set(ControlMode.PercentOutput, rOutput);
		LeftMaster.set(ControlMode.PercentOutput, lOutput);
	}

	//stops motors /shrug
	public void stop()
	{
		RightMaster.stopMotor();
		LeftMaster.stopMotor();
    }

    //Sets speed by taking encoder input and multiplying it by some constant
    public void setSpeed(double lSpeed, double rSpeed)
	{
		double targetVelocityRight = rSpeed * Constants.velocityConstant;
		double targetVelocityLeft = lSpeed * Constants.velocityConstant;
		RightMaster.set(ControlMode.Velocity, targetVelocityRight);
		LeftMaster.set(ControlMode.Velocity, targetVelocityLeft);
	}

    //gives the current of the motors driving the robot
	public void testDrivetrainCurrent()
	{
		System.out.println("Left Motor Current: " + LeftMaster.getOutputCurrent());
		System.out.println("Right Motor Current:" + RightMaster.getOutputCurrent());
	}
    
    //Current limits the drivetrain motors if required
	public void enableCurrentLimiting(double amps)
	{
		LeftMaster.enableCurrentLimit(true);
		RightMaster.enableCurrentLimit(true);
	}
	//Sets the motors to brake mode (This means they stop when you want it to with little deceleration)
	public void setToBrake()
	{
		LeftMaster.setNeutralMode(NeutralMode.Brake);
		RightMaster.setNeutralMode(NeutralMode.Brake);
		LeftSlave.setNeutralMode(NeutralMode.Brake);
		RightMaster.setNeutralMode(NeutralMode.Brake);
	}
    //Sets the motors to Coast mode (This means they have a deceleration when a joystick is in neutral position)
	public void setToCoast()
	{
		LeftMaster.setNeutralMode(NeutralMode.Coast);
		RightMaster.setNeutralMode(NeutralMode.Coast);
        LeftSlave.setNeutralMode(NeutralMode.Coast);
        RightSlave.setNeutralMode(NeutralMode.Coast);
    }

    /*Curvature Drive, also known as "Cheesy Drive."
     *This is an alternate way of using one value to control throttle and one value for rotation. 
     *The rotation argument controls the curvature of the robot's path rather than its rate of heading change. 
     *This makes the robot more controllable at high speeds. Also handles the robot's quick turn functionality - 
     *"quick turn" overrides constant-curvature turning for turn-in-place maneuvers.
     */
    private void curvatureDrive(double throttle, double turn)
	{
		drive.curvatureDrive(throttle, turn, true);	//curvature drive from WPILIB libraries.
	}

    /*if(Constants.kWillToLive <= 0){
     * LeftMaster.set(1);
     * RightMaster.set(1);
     * }
     */
}