/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.huskyrobotics.frc2019.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.sensors.PigeonIMU;

import org.huskyrobotics.frc2019.Constants;
import org.huskyrobotics.frc2019.loops.*;
import org.huskyrobotics.lib.drivers.TalonSRXFactory;
import org.huskyrobotics.lib.drivers.VictorSPXFactory;
import org.huskyrobotics.lib.util.DriveSignal;


import java.util.ArrayList;

public class Drive extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private static final double Encoder_ppr = 4096; //Describes the rate at which our 63r encoders update
  private static Drive m_Instance = new Drive(); //Creates an instance of the Drive command

  //Physical Hardware
  private final TalonSRX m_LeftMaster, m_RightMaster;
  private final VictorSPX m_LeftSlave, m_RightSlave;
  private final Solenoid m_Shifter;

  // Control states

  private PigeonIMU m_Pigeon;

  //Software States
  private boolean m_AutoShift;
  private boolean m_IsHighGear;
  private boolean m_IsBrakeMode;

  private void configureMaster(TalonSRX talon, boolean left) {
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, 100);
    final ErrorCode sensorPresent = talon.configSelectedFeedbackSensor(FeedbackDevice
            .QuadEncoder, 0, 100); //primary closed-loop, 100 ms timeout
    if (sensorPresent != ErrorCode.OK) {
        DriverStation.reportError("Could not detect " + (left ? "left" : "right") + " encoder: " + sensorPresent, false);
    }
    talon.setInverted(!left);
    talon.setSensorPhase(true);
    talon.enableVoltageCompensation(true);
    talon.configVoltageCompSaturation(12.0, 100);
    talon.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_50Ms, 100);
    talon.configVelocityMeasurementWindow(1, 100);
    talon.configClosedloopRamp(Constants.c_DriveVoltageRampRate, 100);
    talon.configNeutralDeadband(0.04, 0);
}
private Drive() {

    // Start all Talons in open loop mode.
    m_LeftMaster = TalonSRXFactory.createDefaultTalon(0);
    configureMaster(m_LeftMaster, true);
    m_LeftSlave = VictorSPXFactory.createDefaultVictorSPX(0);

    m_RightMaster = TalonSRXFactory.createDefaultTalon(1);
    configureMaster((m_RightMaster), false);
    m_RightSlave = VictorSPXFactory.createDefaultVictorSPX(1);

    m_Shifter = Constants.makeSolenoidForId(Constants.c_ShifterSolenoidID);

    m_Pigeon = new PigeonIMU(3);

    //reloadGains();

    //Force solenoid message
    m_IsHighGear = true;
    //setHighGear(false);


    //Force a CAN message
    m_IsBrakeMode = true;
    //setBrakeMode(false);
}
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}