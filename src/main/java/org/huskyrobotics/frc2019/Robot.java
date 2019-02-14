/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.huskyrobotics.frc2019;


import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.huskyrobotics.frc2019.subsystems.*;
import org.huskyrobotics.frc2019.subsystems.drive.*;
import org.huskyrobotics.frc2019.subsystems.drive.FalconLibStuff.FalconDrive;

import org.ghrobotics.lib.debug.LiveDashboard;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Rotation2d;
import org.ghrobotics.lib.mathematics.units.derivedunits.VelocityKt;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import java.util.Map;

import org.huskyrobotics.frc2019.autonomous.Trajectories;
import org.huskyrobotics.frc2019.commands.*;
import org.huskyrobotics.frc2019.commands.UseDrive;
//import org.huskyrobotics.frc2019.subsystems.hatch.*;
//import org.huskyrobotics.frc2019.subsystems.cargo.*;
import org.huskyrobotics.frc2019.inputs.*;
import org.huskyrobotics.frc2019.subsystems.superstructure.*;
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  //public static OI m_oi;
  private PivotArm m_arm;
  //private CargoIO m_cargo;
  //private HatchIO m_hatch;
  public static FalconDrive m_Drive = FalconDrive.getInstance();
  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<Command>();
  //private ShuffleboardTab tab = Shuffleboard.getTab("Commands");

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_Drive.getLocalization().reset(new Pose2d(LengthKt.getFeet(5), LengthKt.getFeet(17), new Rotation2d(0f, 0f, false)));

    Trajectories.generateAllTrajectories();

    m_Drive.init();
    m_Drive.zeroGyro();

    //m_oi = new OI();                                                                                    These all 
    //m_arm = new PivotArm(RobotMap.armMotorPWM, RobotMap.armMotorDIO, RobotMap.armSensor);               error out
    //m_cargo = new CargoIO(RobotMap.cargoMotorPWM, RobotMap.cargoMotorDIO, RobotMap.cargoSensor);
    //m_hatch = new HatchIO(RobotMap.actuatorPortsPWM, RobotMap.actuatorPortsDIO);
    // chooser.addOption("My Auto", new MyAutoCommand());
    SmartDashboard.putData("Auto mode", m_chooser);
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
        
		SmartDashboard.putNumber("Left drivetrian feet per second", feetPerSecond.get(0));
    SmartDashboard.putNumber("Right drivetrian feet per second", feetPerSecond.get(1));

    SmartDashboard.putNumber("Rough Speed", Math.sqrt(Math.pow(feetPerSecond.get(0), 2) + Math.pow(feetPerSecond.get(1), 2)));

    SmartDashboard.putNumber("Left Acceleration", feetPerSecondPerSecond.get(0));
    SmartDashboard.putNumber("Right Aceeleration", feetPerSecondPerSecond.get(0));

    SmartDashboard.putNumber("Rough Acceleration", Math.sqrt(Math.pow(feetPerSecondPerSecond.get(0), 2) + Math.pow(feetPerSecondPerSecond.get(1), 2)));

		SmartDashboard.putNumber("7 feet per second is", m_Drive.getLeft().getModel().fromModel(LengthKt.getFeet(7)).getValue());

		SmartDashboard.putNumber("Current Gyro angle", m_Drive.getGyro());
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
    m_Drive.setNeutralMode(NeutralMode.Coast);
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
    m_autonomousCommand = m_chooser.getSelected();
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
    m_Drive.followTrajectory(Trajectories.Hatch);
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
    m_Drive.zeroGyro();
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
