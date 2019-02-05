/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.huskyrobotics.frc2019.subsystems.cargo;

import org.huskyrobotics.frc2019.subsystems.*;
import org.huskyrobotics.frc2019.subsystems.superstructure.*;

public class CargoMech implements HuskySubsystem
{
  private PivotArm m_arm;
  private CargoIntake m_intake;
  public CargoMech (int ArmMotorPort, int ArmSensorPort, int ClawMotorPort, int ClawSensorPort) {
     m_arm = new PivotArm(ArmMotorPort, ArmSensorPort);
    m_intake = new CargoIntake(ClawMotorPort, ClawSensorPort);
  }
  
  public void onDeactivate () {

  }
  
  public void SetArmTarget (double angle) {
    m_arm.SetTarget(angle);
  }
  
  public void ToggleIntake () {
    m_intake.Toggle();
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
