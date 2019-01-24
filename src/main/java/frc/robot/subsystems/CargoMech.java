/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

public class CargoMech implements HuskySubsystem
{
  private PivotArm _arm;
  private CargoIO _io;
  public CargoMech (int ArmMotorPort, int ArmSensorPort, int ClawMotorPort, int ClawSensorPort)
  {
     _arm = new PivotArm(ArmMotorPort, ArmSensorPort);
    _io = new CargoIO(ClawMotorPort, ClawSensorPort);
  }
  
  public void onDeactivate ()
  {
    _arm.SetActive(false);
    _io.SetActive(false);
  }
  
  public void SetArmTarget (double angle)
  {
    _arm.SetTarget(angle);
  }
  
  public void ToggleIntake ()
  {
    _io.Toggle();
  }
}
