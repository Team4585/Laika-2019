/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.huskyrobotics.frc2019.subsystems.hatch;

import org.huskyrobotics.frc2019.subsystems.*;
import org.huskyrobotics.frc2019.subsystems.superstructure.*;

public class HatchMech implements HuskySubsystem {
  private PivotArm _arm;
  private HatchIO _intake;
  public HatchMech (int ArmMotorPort, int ArmSensorPort, int HatchMotorPort, int HatchSensorPort) {
     _arm = new PivotArm(ArmMotorPort, ArmSensorPort);
    _intake = new HatchIO(HatchMotorPort, HatchSensorPort);
  }
  
  
  public void SetArmTarget (double angle) {
    _arm.SetTarget(angle);
  }
  
  public void release () {
    _intake.release();
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
