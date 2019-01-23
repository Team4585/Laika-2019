/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

public class Cargo extends HuskySubsystem
{
  private PivotArm _arm;
  private CargoIntake _intake;
  public Cargo (int ArmMotorPort, int ArmSensorPort, int ClawMotorPort, int ClawSensorPort)
  {
     _arm = new PivotArm(ArmMotorPort, ArmSensorPort);
    _intake = new CargoIntake(ClawMotorPort, ClawSensorPort);
  }
  
  public void SetArmTarget (double angle)
  {
    _arm.SetTarget(double angle);
  }
  
  public void ToggleIntake ()
  {
    _intake.Toggle();
  }
}
