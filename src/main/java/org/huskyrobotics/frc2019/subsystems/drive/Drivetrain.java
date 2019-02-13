/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.huskyrobotics.frc2019.subsystems.drive;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Solenoid;


import org.huskyrobotics.lib.subsystems.*;
import org.huskyrobotics.frc2019.Constants;
import org.huskyrobotics.frc2019.RobotMap;
import org.huskyrobotics.frc2019.inputs.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.*;

public class Drivetrain extends Subsystem {
    public void initDefaultCommand() {
        //setDefaultCommand(new UseDrivetrain());
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
	}
   //Defining Master Talon SRXs
   private WPI_TalonSRX m_LeftMaster = new WPI_TalonSRX(RobotMap.kLeftMaster);
   private WPI_TalonSRX m_RightMaster = new WPI_TalonSRX(RobotMap.kRightMaster);

   //Defining Slave Victor SPXs
   private VictorSPX m_LeftSlave = new VictorSPX(RobotMap.kLeftSlave);
   private VictorSPX m_RightSlave = new VictorSPX(RobotMap.kRightSlave);
   
   //Creates a double to keep track of the angle gathered by the Pigeon
   public double supposedAngle;


   //Defining Control from WPI Talon SRXs
   public DifferentialDrive drive = new DifferentialDrive(m_LeftMaster, m_RightMaster);

   //Creating an inverted boolean
   Boolean inverted = true;

   //Creating Solenoid object for Shifter
   Solenoid m_Shifter = new Solenoid(0);
   public Boolean m_IsHighGear;

   public static Drivetrain instance;

    public void DTinit() {
       
    // Configure Left Side
        m_LeftMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, Constants.drivetrain.drivePIDIdx, Constants.drivetrain.kTimeoutMs);
        m_LeftMaster.setSensorPhase(true);
        m_LeftMaster.configNominalOutputForward(0, Constants.drivetrain.kTimeoutMs);
        m_LeftMaster.configNominalOutputReverse(0, Constants.drivetrain.kTimeoutMs);
        m_LeftMaster.configPeakOutputForward(1, Constants.drivetrain.kTimeoutMs);
        m_LeftMaster.configPeakOutputReverse(-1,Constants.drivetrain.kTimeoutMs);

    // Configure Right Side 
       m_RightMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, Constants.drivetrain.drivePIDIdx, Constants.drivetrain.kTimeoutMs);
       m_RightMaster.setSensorPhase(true);

       m_RightMaster.configNominalOutputForward(0, Constants.drivetrain.kTimeoutMs);
       m_RightMaster.configNominalOutputReverse(0, Constants.drivetrain.kTimeoutMs);
       m_RightMaster.configPeakOutputForward(1, Constants.drivetrain.kTimeoutMs);
       m_RightMaster.configPeakOutputReverse(-1,Constants.drivetrain.kTimeoutMs);
    
    // Slaving Victors
        m_LeftSlave.follow(m_LeftMaster);
        m_RightSlave.follow(m_RightMaster);
    
    // Overcomplicating Inverted Methods
        m_RightSlave.setInverted(inverted);
        m_RightMaster.setInverted(inverted);
    }
    public void shift() {
        System.out.println("Shifting gears");
        m_Shifter.set(m_IsHighGear);
        if(m_IsHighGear == true) {
            System.out.println("Shifted to High Gear");
        } else {
            System.out.println("Shifted to Low Gear");
        }
    }
    /*public void customArcadeDrive(double xValue, double yValue, Gyro gyro)
	{
		if(yValue != 0 && Math.abs(xValue) < 0.15) //Checks that the xValue is less than 15%, giving some turning deadband
        {
			setSpeed(yValue, yValue);
	 	}
		else if(yValue == 0 && Math.abs(xValue) < 0.15) {//Resets the angle when there's no yValue
			gyro.resetAngle();
			stop();
			supposedAngle = gyro.getYaw();
		} else {
			gyro.resetAngle();
			curvatureDrive(xValue, yValue);
			supposedAngle = gyro.getYaw();
		}
	}*/
    
    //Sets the percent output of each motor
    public void setPercentOutput(double lOutput, double rOutput) {
	    m_RightMaster.set(ControlMode.PercentOutput, rOutput);
		m_LeftMaster.set(ControlMode.PercentOutput, lOutput);
	}

	//stops motors /shrug
	public void stop() {
		m_RightMaster.stopMotor();
		m_LeftMaster.stopMotor();
    }

    //Sets speed by taking encoder input and multiplying it by some constant
    public void setSpeed(double lSpeed, double rSpeed)
	{
    if(m_IsHighGear == true){
		double targetVelocityRight = rSpeed *Constants.drivetrain.kHGvelocityConstant;
		double targetVelocityLeft = lSpeed * Constants.drivetrain.kHGvelocityConstant;
		m_RightMaster.set(ControlMode.Velocity, targetVelocityRight);
        m_LeftMaster.set(ControlMode.Velocity, targetVelocityLeft);
      }else{
        double targetVelocityRight = rSpeed * Constants.drivetrain.kLGvelocityConstant;
		double targetVelocityLeft = lSpeed * Constants.drivetrain.kLGvelocityConstant;
		m_RightMaster.set(ControlMode.Velocity, targetVelocityRight);
        m_LeftMaster.set(ControlMode.Velocity, targetVelocityLeft);
        }
	}

    //gives the current of the motors driving the robot
	public void testDrivetrainCurrent() {
		System.out.println("Left Motor Current: " + m_LeftMaster.getOutputCurrent());
		System.out.println("Right Motor Current:" + m_RightMaster.getOutputCurrent());
	}
    
    //Current limits the drivetrain motors if required
	public void enableCurrentLimiting(double amps) {
		m_LeftMaster.enableCurrentLimit(true);
		m_RightMaster.enableCurrentLimit(true);
	}
	//Sets the motors to brake mode (This means they stop when you want it to with little deceleration)
	public void setToBrake() {
		m_LeftMaster.setNeutralMode(NeutralMode.Brake);
		m_RightMaster.setNeutralMode(NeutralMode.Brake);
		m_LeftSlave.setNeutralMode(NeutralMode.Brake);
		m_RightSlave.setNeutralMode(NeutralMode.Brake);
	}
    //Sets the motors to Coast mode (This means they have a deceleration when a joystick is in neutral position)
	public void setToCoast() {
		m_LeftMaster.setNeutralMode(NeutralMode.Coast);
		m_RightMaster.setNeutralMode(NeutralMode.Coast);
        m_LeftSlave.setNeutralMode(NeutralMode.Coast);
        m_RightSlave.setNeutralMode(NeutralMode.Coast);
    }

    /*Curvature Drive, also known as "Cheesy Drive."
     *This is an alternate way of using one value to control throttle and one value for rotation. 
     *The rotation argument controls the curvature of the robot's path rather than its rate of heading change. 
     *This makes the robot more controllable at high speeds. Also handles the robot's quick turn functionality - 
     *"quick turn" overrides constant-curvature turning for turn-in-place maneuvers.
     */
    public void curvatureDrive(double throttle, double turn) {
		drive.curvatureDrive(throttle, turn, true);	//curvature drive from WPILIB libraries.
    }
    
    public static Drivetrain getInstance()
    {
        if (instance == null)
            instance = new Drivetrain();

        return instance;
    }
    public void meirl(){
    if(Constants.kWillToLive <= 0){
        m_LeftMaster.set(1);
        m_RightMaster.set(1);
        }
    }
}