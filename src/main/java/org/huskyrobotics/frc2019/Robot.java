/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.huskyrobotics.frc2019;


import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.huskyrobotics.frc2019.subsystems.cargo.CargoIO;
import org.huskyrobotics.frc2019.subsystems.drive.FalconLibStuff.FalconDrive;
import org.huskyrobotics.frc2019.subsystems.superstructure.PivotArm;
import org.ghrobotics.lib.debug.LiveDashboard;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Rotation2d;
import org.ghrobotics.lib.mathematics.units.derivedunits.VelocityKt;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import org.huskyrobotics.frc2019.Constants.limelight;
import org.huskyrobotics.frc2019.FalconAuto.*;
import org.huskyrobotics.frc2019.auto.Auto;
import org.huskyrobotics.frc2019.commands.Auto.*;
//import org.huskyrobotics.frc2019.commands.Auto.CargoRoutine;
import org.huskyrobotics.frc2019.inputs.*;
import org.huskyrobotics.frc2019.inputs.Encoder.EncoderMode;
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public Robot(){
    super(0.025d);
  }
  public static OI m_Oi = new OI(0,1);
  //public static CargoIO m_Cargo = new CargoIO(RobotMap.kIntake);
  //private HatchIO m_hatch;
  public static FalconDrive m_Drive = FalconDrive.getInstance();
  public static Vision m_Limelight = new Vision();
  //public static PivotArm m_Pivot = new PivotArm(RobotMap.kPivotMaster, EncoderMode.QuadEncoder);
  private Compressor m_Compressor = new Compressor();
  private Boolean m_HasTarget;
  Command m_autonomousCommand;
  SendableChooser<Command> Auto;
  //private ShuffleboardTab tab = Shuffleboard.getTab("Commands");

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_Drive.getLocalization().reset(new Pose2d(LengthKt.getFeet(5), LengthKt.getFeet(17), new Rotation2d(0f, 0f, false)));
    m_Drive.init();
    m_Drive.zeroGyro();
    System.out.println("Robot is Zeroed and init'd");
    
    
    

    Trajectories.generateAllTrajectories();
    m_Compressor.setClosedLoopControl(true);                                                                                        
    //m_cargo = new CargoIO(RobotMap.cargoMotorPWM, RobotMap.cargoMotorDIO, RobotMap.cargoSensor);
    //m_hatch = new HatchIO(RobotMap.actuatorPortsPWM, RobotMap.actuatorPortsDIO);
    Auto = new SendableChooser<Command>();
    Auto.addOption("My Auto", new CargoRoutine('m', 'l'));
    Auto.addOption("Test", new TestAuto());
    Auto.addOption("Drive Straight", new DriveStraight(2));
    SmartDashboard.putData("Auto", Auto);



  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    m_Drive.getLocalization().update();


    SmartDashboard.putNumber("Robot X (feet) ", m_Drive.getLocalization().getRobotPosition().getTranslation().getX().getFeet());
    SmartDashboard.putNumber("Robot Y (feet) ", m_Drive.getLocalization().getRobotPosition().getTranslation().getY().getFeet());
    SmartDashboard.putNumber("Robot Heading (degrees)", m_Drive.getLocalization().getRobotPosition().getRotation().getDegree());

		LiveDashboard.INSTANCE.setRobotX(m_Drive.getLocalization().getRobotPosition().getTranslation().getX().getFeet());
		LiveDashboard.INSTANCE.setRobotY(m_Drive.getLocalization().getRobotPosition().getTranslation().getY().getFeet());
		LiveDashboard.INSTANCE.setRobotHeading(m_Drive.getLocalization().getRobotPosition().getRotation().getRadian());

		SmartDashboard.putNumber("Left talon speed", m_Drive.getLeft().getFeetPerSecond());
		SmartDashboard.putNumber("Left talon error", m_Drive.getLeft().getClosedLoopError().getFeet());
		SmartDashboard.putNumber("Right talon speed", m_Drive.getRight().getFeetPerSecond());
		SmartDashboard.putNumber("Right talon error", m_Drive.getRight().getClosedLoopError().getFeet());

		List<Double> feetPerSecond = Arrays.asList(
				VelocityKt.getFeetPerSecond(m_Drive.getLeft().getVelocity()),
				VelocityKt.getFeetPerSecond(m_Drive.getRight().getVelocity()));
		List<Double> feetPerSecondPerSecond = Arrays.asList(
				(VelocityKt.getFeetPerSecond(m_Drive.getLeft().getVelocity()) - m_Drive.lastVelocity.get(0)) / 0.02d,
        (VelocityKt.getFeetPerSecond(m_Drive.getRight().getVelocity()) - m_Drive.lastVelocity.get(0)) / 0.02d);
        
		SmartDashboard.putNumber("Left drivetrain feet per second", feetPerSecond.get(0));
    SmartDashboard.putNumber("Right drivetrain feet per second", feetPerSecond.get(1));

    SmartDashboard.putNumber("Rough Speed", Math.sqrt(Math.pow(feetPerSecond.get(0), 2) + Math.pow(feetPerSecond.get(1), 2)));

    SmartDashboard.putNumber("Left Acceleration", feetPerSecondPerSecond.get(0));
    SmartDashboard.putNumber("Right Aceeleration", feetPerSecondPerSecond.get(0));

    SmartDashboard.putNumber("Rough Acceleration", Math.sqrt(Math.pow(feetPerSecondPerSecond.get(0), 2) + Math.pow(feetPerSecondPerSecond.get(1), 2)));

		SmartDashboard.putNumber("7 feet per second is", m_Drive.getLeft().getModel().fromModel(LengthKt.getFeet(7)).getValue());

    SmartDashboard.putNumber("Current Gyro angle", m_Drive.getGyro());
    
    double[] limelightdata = m_Limelight.getData();
    if(limelightdata[0] == 1){
      m_HasTarget = true;
    }

		SmartDashboard.putNumber("Vision targets?", limelightdata[0]);
		SmartDashboard.putNumber("Horizontal offset", limelightdata[1]);
		SmartDashboard.putNumber("Vertical offset", limelightdata[2]);
		SmartDashboard.putNumber("Target area", limelightdata[3]);
		SmartDashboard.putNumber("Target skew", limelightdata[4]);
		SmartDashboard.putNumber("Vision pipeline latency", limelightdata[5]);
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
    m_Drive.getLocalization().reset(new Pose2d(LengthKt.getFeet(5), LengthKt.getFeet(17), new Rotation2d(0f, 0f, false)));
    m_Drive.setNeutralMode(NeutralMode.Coast);
    m_Drive.setLowGear();
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = Auto.getSelected();
    m_Drive.zeroEncoders();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
   
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
    

  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
