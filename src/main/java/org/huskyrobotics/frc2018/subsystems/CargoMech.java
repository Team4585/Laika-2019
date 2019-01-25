/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.huskyrobotics.frc2018.subsystems;

public class CargoMech implements HuskySubsystem
{
  private PivotArm _arm;
  private CargoIO _slurp;
  public CargoMech (int ArmMotorPort, int ArmSensorPort, int ClawMotorPort, int ClawSensorPort) {
    _arm = new PivotArm(ArmMotorPort, ArmSensorPort);
    _slurp = new CargoIO(ClawMotorPort, ClawSensorPort);
  }
  
  public void onDeactivate () {

  }
  
  public void SetArmTarget (double angle) {
    _arm.SetTarget(angle);
  }
  
  public void ToggleIntake () {
    _intake.Toggle();
  }

  @Override
  public void autoInit() {

  }

  @Override
  public void doAuto() {

  }

  @Override
  public void teleopInit() {

  }

  @Override
  public void doTeleop() {

  }
}
