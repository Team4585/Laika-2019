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
  private CargoIO _intake;
  public CargoMech (int ArmMotorPort, int ArmSensorPort, int ClawMotorPort, int ClawSensorPort) {
    _arm = new PivotArm(ArmMotorPort, ArmSensorPort);
    _intake = new CargoIO(ClawMotorPort, ClawSensorPort);
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
