/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.huskyrobotics.frc2019.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import org.huskyrobotics.frc2019.subsystems.drive.Drivetrain;

/**
 * Add your docs here.
 */
public class Shift extends InstantCommand {
  /**
   * Add your docs here.
   */
  Drivetrain m_Drive = new Drivetrain();
  public Shift() {
    super();
    requires(m_Drive);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    m_Drive.shift();
  }

}
