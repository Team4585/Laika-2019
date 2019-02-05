/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.huskyrobotics.frc2019.subsystems.cargo;

import edu.wpi.first.wpilibj.NidecBrushless;
import edu.wpi.first.wpilibj.DigitalInput;

import org.huskyrobotics.frc2019.subsystems.*;

public class CargoIO implements HuskySubsystem {
	private NidecBrushless m_motor;
	private DigitalInput m_limitSwitch;
	
	private boolean hasBall = false;
	
	private double maxSpeed = 1.0;

	//private SomeSensor _CheckSensor;
	public CargoIO (int motorChannelPWM, int  motorChannelDio, int sensorChannel) {
		m_motor = new NidecBrushless(motorChannelPWM, motorChannelDio);
		m_limitSwitch = new DigitalInput(sensorChannel);
	}
	
	public void activate () {
		if (hasBall) {
			output();
			hasBall = false;
		} else {
			intake();
		}
	}
	
	private void intake () {
		m_motor.set(maxSpeed);
	}
	
	private void output () {
		m_motor.set(-maxSpeed);
	}
	
	private void init() {
		m_motor.stopMotor();
		hasBall = false;
	}
	private void do() {
		hasBall = m_limitSwitch.get();
	}

	@Override
	public void autoInit() {
		init();
	}

	@Override
	public void doAuto() {
		do();
	}

	@Override
	public void teleopInit() {
		init();
	}

	@Override
	public void doTeleop() {
		do();
	}
}
